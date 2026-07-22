#include "io/camera.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

int main(int argc, char * argv[])
{
  namespace fs = std::filesystem;
  auto exe_path = fs::absolute(fs::path(argv[0]));
  auto default_config_path = (exe_path.parent_path() / ".." / "configs" / "camera.yaml").lexically_normal().string();
  const std::string keys =
    "{help h usage ? |                     | 输出命令行参数说明}"
    "{config c      | " + default_config_path + " | yaml配置文件路径 }"
    "{d display     |                     | 显示视频流       }";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  auto config_path = cli.get<std::string>("config");
  auto display = cli.has("display") || (argc == 1);

  io::Camera camera(config_path);

  if (display) {
    cv::namedWindow("Galaxy Preview", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Galaxy Preview", 1280, 720);
  }

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, timestamp);

    if (img.empty()) {
      tools::logger()->warn("Empty frame received from camera");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;
    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (display) {
      cv::Mat show = img.clone();
      cv::putText(show, "Press q or ESC to quit", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 0), 2);
      cv::putText(show, fmt::format("FPS: {:.1f}", 1 / dt), cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 0), 2);
      cv::imshow("Galaxy Preview", show);
      int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q' || key == 27) {
        break;
      }
    }
  }

  return 0;
}
