#include <chrono>
#include <filesystem>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/exiter.hpp"

const std::string keys =
  "{help h usage ? |                                    | 输出命令行参数说明}"
  "{@config-path   | configs/desktop_gimbal.yaml | yaml配置文件路径 }"
  "{output-dir  o  | assets/calib_images                 | 输出文件夹路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }

  auto config_path = cli.get<std::string>(0);
  auto output_dir = cli.get<std::string>("output-dir");

  // 创建输出目录
  std::filesystem::create_directories(output_dir);

  tools::Exiter exiter;
  io::Camera camera(config_path);

  // 棋盘格参数：11×8 格子 → 10×7 内角点 (屏幕版SVG)
  const cv::Size pattern_size(10, 7);
  fmt::print("   棋盘格: {}x{} 内角点 (11x8 格子)\n", pattern_size.width, pattern_size.height);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  int count = 0, stable_count = 0;
  bool display_green = false;
  // 读取已有文件以确定起始编号
  for (const auto & entry : std::filesystem::directory_iterator(output_dir)) {
    auto name = entry.path().stem().string();
    try { count = std::max(count, std::stoi(name)); } catch (...) {}
  }

  fmt::print("📷 标定图像采集程序\n");
  fmt::print("   棋盘格: {}x{} 内角点\n", pattern_size.width, pattern_size.height);
  fmt::print("   保存至: {}/\n", output_dir);
  fmt::print("   按键: [s]保存  [q]退出\n\n");

  while (!exiter.exit()) {
    camera.read(img, t);

    // 每帧检测，带迟滞消除闪烁
    std::vector<cv::Point2f> corners;
    auto found = cv::findChessboardCorners(img, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    // 迟滞：变绿需连续3次检测到，变红需连续3次丢失
    // cap ±6，确保最长 0.6s (9帧@15fps) 响应
    if (found) {
      stable_count = std::min(stable_count + 1, 6);
    } else {
      stable_count = std::max(stable_count - 1, -6);
    }
    if (!display_green && stable_count >= 3) display_green = true;
    if (display_green && stable_count <= -3) display_green = false;

    auto display = img.clone();
    if (display_green) {
      cv::drawChessboardCorners(display, pattern_size, corners, found);
      cv::putText(display, "CHESSBOARD DETECTED", {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
    } else {
      cv::putText(display, "No chessboard", {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
    }
    cv::putText(display, fmt::format("Saved: {}", count), {10, 60},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 2);
    cv::putText(display, "[s]ave  [q]uit", {10, display.rows - 20},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {200, 200, 200}, 1);

    cv::resize(display, display, {}, 0.4, 0.4);
    cv::imshow("Calibration Capture", display);
    auto key = cv::waitKey(1);

    // 保存时用 found（实际检测到才保存，不看显示状态）
    if (key == 'q') break;
    if (key == 's' && found) {
      // 亚像素精化后保存
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
      count++;
      auto path = fmt::format("{}/{}.jpg", output_dir, count);
      cv::imwrite(path, img);
      fmt::print("  ✅ [{}] 已保存: {}\n", count, path);
    } else if (key == 's') {
      fmt::print("  ⚠️  未检测到棋盘格，不保存\n");
    }
  }

  fmt::print("\n📊 共保存 {} 张标定图像\n", count);
  fmt::print("   下一步: calibrate_camera --input-folder {}\n", output_dir);
  return 0;
}
