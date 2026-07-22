#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?      |                               | 打印帮助}"
  "{@config-path        | configs/desktop_gimbal.yaml | 配置文件}"
  "{no-gimbal     |      | 视觉调试模式}";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto no_gimbal = cli.has("no-gimbal");
  if (cli.has("help") || config_path.empty()) { cli.printMessage(); return 0; }

  tools::Exiter exiter;

  io::Gimbal * gimbal_ptr = nullptr;
  std::unique_ptr<io::Gimbal> gimbal;
  if (!no_gimbal) {
    gimbal = std::make_unique<io::Gimbal>(config_path);
    gimbal_ptr = gimbal.get();
  }

  io::Camera camera(config_path);

  // ── 参数 ──
  constexpr float KP_YAW = 0.08f, KP_PITCH = 0.05f;
  constexpr float MAX_CMD = 30.0f;
  constexpr int MIN_AREA = 500;
  constexpr float DEADBAND = 10.0f;
  constexpr float SMOOTH = 0.3f;
  constexpr float BG_ALPHA = 0.02f;  // 背景更新速率（慢=忽略云台移动）

  cv::Mat bg_gray;          // 滑动平均背景
  bool bg_initialized = false;

  float smooth_tx = 0, smooth_ty = 0;
  bool has_prev_target = false;
  int lost_count = 0;

  tools::logger()->info("[SimpleTracker] Entering main loop.");

  while (!exiter.exit()) {
    cv::Mat frame;
    std::chrono::steady_clock::time_point t;
    camera.read(frame, t);

    auto mode = io::GimbalMode::AUTO_AIM;
    if (gimbal_ptr) mode = gimbal_ptr->mode();
    if (mode == io::GimbalMode::IDLE) mode = io::GimbalMode::AUTO_AIM;

    float img_cx = frame.cols / 2.0f, img_cy = frame.rows / 2.0f;

    // ── 滑动平均背景差法 ──
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, {15, 15}, 0);
    
    if (!bg_initialized) {
      bg_gray = gray.clone();
      bg_initialized = true;
    }
    
    // 背景差
    cv::Mat diff, fg;
    cv::absdiff(gray, bg_gray, diff);
    cv::threshold(diff, fg, 20, 255, cv::THRESH_BINARY);
    cv::dilate(fg, fg, cv::Mat(), cv::Point(-1,-1), 2);
    cv::morphologyEx(fg, fg, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 1);
    
    // 慢速更新背景（云台移动不会污染背景）
    cv::addWeighted(bg_gray, 1.0 - BG_ALPHA, gray, BG_ALPHA, 0, bg_gray);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool has_target = false;
    float tx = 0, ty = 0;
    int best_idx = -1;
    double best_area = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
      double area = cv::contourArea(contours[i]);
      if (area > best_area && area > MIN_AREA) {
        best_area = area;
        best_idx = i;
      }
    }
    if (best_idx >= 0) {
      cv::Moments m = cv::moments(contours[best_idx]);
      if (m.m00 > 0) {
        tx = (float)(m.m10 / m.m00);
        ty = (float)(m.m01 / m.m00);
        has_target = true;
        cv::drawContours(frame, contours, best_idx, cv::Scalar(0, 255, 0), 2);
        cv::circle(frame, cv::Point2i((int)tx, (int)ty), 8, cv::Scalar(0, 0, 255), -1);
        char buf[64];
        snprintf(buf, sizeof(buf), "TARGET (%.0f,%.0f)", tx, ty);
        cv::putText(frame, buf, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      }
    }

    // ── 目标丢失处理 + 平滑 ──
    if (has_target) {
      if (!has_prev_target) {
        smooth_tx = tx; smooth_ty = ty;
      } else {
        smooth_tx += (tx - smooth_tx) * SMOOTH;
        smooth_ty += (ty - smooth_ty) * SMOOTH;
      }
      tx = smooth_tx; ty = smooth_ty;
      has_prev_target = true;
      lost_count = 0;
    } else {
      lost_count++;
      // 丢失超过30帧才放弃跟踪
      if (lost_count > 30) has_prev_target = false;
    }

    // ── P控制 ──
    float yaw_cmd = 0, pitch_cmd = 0;
    if (has_target || has_prev_target) {
      float dx = tx - img_cx, dy = ty - img_cy;
      if (std::abs(dx) > DEADBAND) yaw_cmd = KP_YAW * dx;
      if (std::abs(dy) > DEADBAND) pitch_cmd = KP_PITCH * dy;
      yaw_cmd = std::clamp(yaw_cmd, -MAX_CMD, MAX_CMD);
      pitch_cmd = std::clamp(pitch_cmd, -MAX_CMD, MAX_CMD);
    }

    // 发云台
    if (gimbal_ptr && (has_target || has_prev_target) && mode != io::GimbalMode::IDLE) {
      gimbal_ptr->send(true, false, yaw_cmd * 0.0174533f, 0, 0, pitch_cmd * 0.0174533f, 0, 0);
      tools::logger()->info("[CMD] yaw={:.2f}deg pitch={:.2f}deg", yaw_cmd, pitch_cmd);
    } else if (gimbal_ptr) {
      gimbal_ptr->send(false, false, 0, 0, 0, 0, 0, 0);
    }

    // ── 显示 ──
    cv::putText(frame, (has_target ? "LOCK" : lost_count > 0 ? "SEARCH" : "INIT"),
                {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    if (no_gimbal) cv::putText(frame, "NO-GIMBAL", {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);

    cv::resize(frame, frame, {}, 0.5, 0.5);
    cv::imshow("desktop_gimbal", frame);
    if (cv::waitKey(1) == 'q') break;
  }

  tools::logger()->info("[SimpleTracker] Shutdown.");
  return 0;
}
