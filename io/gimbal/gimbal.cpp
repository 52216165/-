#include "gimbal.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <algorithm>
#include <cstdio>  //新增
#include <cstring> //新增2
#include <filesystem>
#include <vector>
#include <fmt/ranges.h>

namespace io
{
namespace {
constexpr bool kVerboseSerialHexLog = false;

std::string resolve_serial_port(const std::string & configured_port)
{
  namespace fs = std::filesystem;

  if (!configured_port.empty() && fs::exists(configured_port)) {
    return configured_port;
  }

  static const std::vector<std::string> preferred_ports = {
    "/dev/gimbal",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
  };

  for (const auto & port : preferred_ports) {
    if (fs::exists(port)) {
      return port;
    }
  }

  std::vector<std::string> detected_ports;
  try {
    for (const auto & entry : fs::directory_iterator("/dev")) {
      if (!entry.is_character_file()) {
        continue;
      }
      const auto name = entry.path().filename().string();
      if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
        detected_ports.push_back(entry.path().string());
      }
    }
  } catch (...) {
    return configured_port;
  }

  if (detected_ports.empty()) {
    return configured_port;
  }

  std::sort(detected_ports.begin(), detected_ports.end());
  return detected_ports.front();
}
}

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");//读取配置文件中的com_port
  auto resolved_port = resolve_serial_port(com_port);
  if (resolved_port != com_port) {
    tools::logger()->warn(
      "[Gimbal] Configured com_port '{}' not found, fallback to '{}'.", com_port, resolved_port);
  }

  try {
    serial_.setPort(resolved_port);//设置串口
    serial_.setBaudrate(115200);//设置波特率 115200 bps
    
    // 低延迟优先：43字节@115200bps约3.7ms，给适度余量
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // 50ms 总超时
    timeout.inter_byte_timeout = 5; // 字节间超时 5ms
    serial_.setTimeout(timeout);
    
    serial_.open();
    
    // 初始化时清空可能的积压数据
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) {
        serial_.flushInput();
        serial_.flushOutput();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch (...) {}
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
  tx_thread_ = std::thread(&Gimbal::tx_thread, this);

  queue_.pop();//我们的gimbal的队列是1000——必须先收到至少一帧合法姿态数据，系统才继续往下跑
  tools::logger()->info("[Gimbal] First q received.");//如果没有下文，说明：1、没有数据 2、我并没有收到合法的姿态数据
}

/*析构函数，用于退出*/
Gimbal::~Gimbal()
{
  quit_ = true;
  tx_queue_.push(VisionToGimbal{});  // 唤醒发送线程安全退出
  if (thread_.joinable()) thread_.join();
  if (tx_thread_.joinable()) tx_thread_.join();
  serial_.close();
}

/*获取云台模式——0: 空闲, 1: 自瞄, 2: 小符, 3: 大符*/
GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

/*获取云台状态——*/
GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

/*获取诊断信息 - 用于监测串口实时性和可靠性*/
GimbalDiagnostics Gimbal::diagnostics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  GimbalDiagnostics diag;
  
  // 基础信息
  diag.serial_ok = serial_.isOpen();
  auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  auto last_rx_ns = last_rx_ok_ns_.load();
  
  // 计算接收延迟（距离上一次成功接收）
  if (last_rx_ns > 0) {
    diag.rx_latency_ms = static_cast<double>(now_ns - last_rx_ns) / 1e6;
  }
  
  // 帧率和错误率通过日志统计，这里提供基本指标
  // 在实际使用中，应在 read_thread 中定期打印这些数据
  
  return diag;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";//默认
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // Lracking 下位机协议：绝对角度（度），非增量
  // 传入的 VisionToGimbal 已经是度、绝对角度，不做额外转换
  auto gs = state();

  if (VisionToGimbal.mode == 0) { // mode=0对应IDLE
    VisionToGimbal.yaw = 0.0f;
    VisionToGimbal.pitch = 0.0f;
  }

  // 异步发送：控制线程只入队，不阻塞在串口写上
  tx_queue_.push(VisionToGimbal);
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  // Lracking 下位机协议：绝对角度（度），非增量
  auto yaw_deg = yaw * 57.29578f;    // rad → deg
  auto pitch_deg = pitch * 57.29578f;

  if (!control) {
    // Idle 模式时下位机会保持当前位置，角度值被忽略
    yaw_deg = 0.0f;
    pitch_deg = 0.0f;
  }

  VisionToGimbal pkt;
  pkt.mode = control ? (fire ? 2 : 1) : 0;
  pkt.yaw = yaw_deg;
  pkt.yaw_vel = yaw_vel;        // 保留，下位机可作为前馈
  pkt.yaw_acc = yaw_acc;
  pkt.pitch = pitch_deg;
  pkt.pitch_vel = pitch_vel;
  pkt.pitch_acc = pitch_acc;

  // 异步发送：控制线程只入队，不阻塞在串口写上
  tx_queue_.push(pkt);
}

void Gimbal::tx_thread()
{
  while (true) {
    VisionToGimbal pkt;
    tx_queue_.pop(pkt);
    if (quit_) break;

    // 背压时只发最新控制量，丢弃过期命令以降低控制滞后
    VisionToGimbal newest = pkt;
    while (!tx_queue_.empty()) {
      tx_queue_.pop(newest);
    }

    if (kVerboseSerialHexLog) {
      std::string hex;
      hex.reserve(sizeof(newest) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&newest);
      for (size_t i = 0; i < sizeof(newest); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug("[Gimbal] tx frame ({} bytes): {}", sizeof(newest), hex);
    }

    auto tx_start = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    try {
      serial_.write(reinterpret_cast<uint8_t *>(&newest), sizeof(newest));
      auto tx_end = std::chrono::steady_clock::now();
      double tx_latency = std::chrono::duration<double, std::milli>(tx_end - tx_start).count();
      if (tx_latency > 5.0) {
        tools::logger()->warn("[Gimbal] High TX latency: {:.2f}ms", tx_latency);
      }
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
   tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}


void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  uint8_t buffer[64];          // 最大帧大小
  uint8_t stream_buf[1024] = {0};
  size_t stream_len = 0;
  int error_count = 0;
  bool has_received_data = false; // 核心：是否已收到云台的有效回调数据
  int no_data_count = 0;

  while (!quit_) {
    if (error_count > 100) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many consecutive errors (>100), attempting to reconnect...");
      reconnect();
      continue;
    }

    // 1) 批量读取串口流（减少锁竞争和系统调用次数）
    size_t nread = 0;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        size_t avail = serial_.available();
        if (avail > 0) {
          size_t room = sizeof(stream_buf) - stream_len;
          if (room == 0) {
            // 缓冲满了说明长期没同步上，保留末尾窗口避免丢帧头
            std::memmove(stream_buf, stream_buf + sizeof(stream_buf) - 64, 64);
            stream_len = 64;
            room = sizeof(stream_buf) - stream_len;
          }
          size_t to_read = std::min(avail, room);
          nread = serial_.read(stream_buf + stream_len, to_read);
          stream_len += nread;
        }
      } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Failed to read serial stream: {}", e.what());
      }
    }

    if (nread == 0) {
      no_data_count++;
      if (no_data_count > 4000) {
        error_count++;
        no_data_count = 0;
        tools::logger()->warn("[Gimbal] No serial data for extended period, triggering reconnect");
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    no_data_count = 0;

    // 2) 在流缓冲中找帧头+完整帧
    // 先找出所有帧头位置
    std::vector<size_t> head_positions;
    for (size_t i = 1; i < stream_len; ++i) {
      if (stream_buf[i - 1] == 0x5A && stream_buf[i] == 0xA5) {
        head_positions.push_back(i - 1);
      }
    }

    if (head_positions.empty()) {
      // 没找到帧头，保留最后一个字节作为下次匹配窗口
      stream_buf[0] = stream_buf[stream_len - 1];
      stream_len = 1;
      continue;
    }

    // 使用第一个帧头，检查是否有完整的一帧数据
    size_t head_pos = head_positions[0];
    if (stream_len - head_pos < static_cast<size_t>(frame_size_)) {
      // 数据还不够一帧，先对齐到帧头
      if (head_pos > 0) {
        std::memmove(stream_buf, stream_buf + head_pos, stream_len - head_pos);
        stream_len -= head_pos;
      }
      continue;
    }

    // 读取一帧
    std::memcpy(buffer, stream_buf + head_pos, frame_size_);
    size_t consumed = head_pos + frame_size_;
    if (stream_len > consumed) {
      std::memmove(stream_buf, stream_buf + consumed, stream_len - consumed);
      stream_len -= consumed;
    } else {
      stream_len = 0;
    }
    
    // 3. 不检查帧尾——固件实际不发送 0x7F 0xFE
    
    // 解析并靠合理性检查过滤坏帧
    if (error_count > 0) error_count--;

    // 4. 解析数据包
    // 已知正确部分：head(2)+mode(1)+q(16)+yaw(4) = 23字节
    // 其余字段填充0（固件帧结构可能小于43字节）
    std::memset(&rx_data_, 0, sizeof(rx_data_));
    std::memcpy(&rx_data_, buffer, std::min(static_cast<size_t>(frame_size_), sizeof(rx_data_)));
    // 只保留前23字节的可靠数据，其余清零防误读
    if (frame_size_ < 23) {
      error_count++;
      continue;
    }
    auto t = std::chrono::steady_clock::now();

    // 数据合理性检查（只检查 yaw，±180°范围内通过）
    float yaw_val = rx_data_.yaw;
    if (std::isnan(yaw_val) || std::abs(yaw_val) > 200.0f) {
      error_count++;
      continue;
    }

    // 日志（仅在启动时/错误恢复后打印，避免日志爆炸）
    if (kVerboseSerialHexLog && !has_received_data) {
      std::string hex;
      hex.reserve(sizeof(rx_data_) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&rx_data_);
      for (size_t i = 0; i < sizeof(rx_data_); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug("[Gimbal] rx frame ({} bytes): {}", sizeof(rx_data_), hex);
    }
    
    error_count = 0;
    has_received_data = true;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      //yaw可能需要取得反值——原来的是下面两行，改后去取了相反值
      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      // state_.yaw =  -rx_data_.yaw;
      // state_.yaw_vel = -rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;

      switch (rx_data_.mode) {
        case 0:
          mode_ = GimbalMode::IDLE;
          break;
        case 1:
          mode_ = GimbalMode::AUTO_AIM;
          break;
        case 2:
          mode_ = GimbalMode::SMALL_BUFF;
          break;
        case 3:
          mode_ = GimbalMode::BIG_BUFF;
          break;
        default:
          mode_ = GimbalMode::IDLE;
          tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
          break;
      }
    }

    // 记录成功接收时间戳（用于诊断延迟）
    last_rx_ok_ns_.store(t.time_since_epoch().count());
    
    queue_.push({q, t});
    
    // 定期打印诊断信息（降低日志频率）
    static int diag_counter = 0;
    if (++diag_counter >= 200) {
      diag_counter = 0;
      auto diag = diagnostics();
      tools::logger()->info(
        "[Gimbal] Diag: latency={:.2f}ms, serial_ok={}",
        diag.rx_latency_ms,
        diag.serial_ok ? "yes" : "no");
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 15;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) {
        serial_.flushInput();
        serial_.flushOutput();
        serial_.close();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (...) {
    }

    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_.open();
      
      // 重新配置超时（与初始化一致）
      serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
      timeout.inter_byte_timeout = 5;
      serial_.setTimeout(timeout);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      serial_.flushInput();
      serial_.flushOutput();
      
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

}  // namespace io