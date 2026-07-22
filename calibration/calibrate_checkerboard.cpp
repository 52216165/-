#include <yaml-cpp/yaml.h>
#include <fmt/core.h>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>

const std::string keys =
  "{help h usage ?   |                        | 输出命令行参数说明}"
  "{@input-folder    | assets/calib_images     | 标定图像文件夹路径 }"
  "{pattern-cols     | 8                       | 棋盘格内角点列数 }"
  "{pattern-rows     | 5                       | 棋盘格内角点行数 }"
  "{square-size      | 25                      | 格子边长 (mm) }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }
  auto input_folder = cli.get<std::string>(0);
  auto pattern_cols = cli.get<int>("pattern-cols");
  auto pattern_rows = cli.get<int>("pattern-rows");
  auto square_size = cli.get<double>("square-size");

  cv::Size pattern_size(pattern_cols, pattern_rows);
  fmt::print("📐 棋盘格标定\n");
  fmt::print("   内角点: {}x{} (格子 {}x{})\n", pattern_cols, pattern_rows,
             pattern_cols + 1, pattern_rows + 1);
  fmt::print("   格子大小: {}mm\n", square_size);
  fmt::print("   输入文件夹: {}\n\n", input_folder);

  // 3D 棋盘格角点坐标 (Z=0 平面)
  std::vector<cv::Point3f> obj;
  for (int r = 0; r < pattern_rows; r++)
    for (int c = 0; c < pattern_cols; c++)
      obj.emplace_back(c * square_size, r * square_size, 0.0f);

  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  cv::Size img_size;
  int loaded = 0, skipped = 0;

  // 遍历文件夹中的 jpg/png 图片
  for (const auto & entry : std::filesystem::directory_iterator(input_folder)) {
    auto ext = entry.path().extension().string();
    if (ext != ".jpg" && ext != ".png" && ext != ".jpeg") continue;

    auto img = cv::imread(entry.path().string());
    if (img.empty()) { skipped++; continue; }

    if (img_size == cv::Size(0, 0)) img_size = img.size();

    std::vector<cv::Point2f> corners;
    auto found = cv::findChessboardCorners(img, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (found) {
      // 亚像素精化
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
      img_points.push_back(corners);
      obj_points.push_back(obj);
      loaded++;
      fmt::print("  ✅ [{}] {}\n", loaded, entry.path().filename().string());
    } else {
      skipped++;
      fmt::print("  ❌ 未检测到: {}\n", entry.path().filename().string());
    }
  }

  fmt::print("\n📊 成功加载: {} / {} 张\n", loaded, loaded + skipped);
  if (loaded < 5) {
    fmt::print("⚠️  至少需要 5 张有效标定图像，请拍摄更多角度\n");
    return 1;
  }

  // 相机标定
  cv::Mat camera_matrix = cv::initCameraMatrix2D(obj_points, img_points, img_size);
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;

  double rms = cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, dist_coeffs, rvecs, tvecs,
    cv::CALIB_FIX_K3,  // 视场角不大，不需要 k3
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));

  // 计算重投影误差
  double total_error = 0;
  size_t total_pts = 0;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, reprojected);
    total_error += cv::norm(img_points[i], reprojected, cv::NORM_L2);
    total_pts += obj_points[i].size();
  }
  double mean_error = total_error / total_pts;

  fmt::print("\n✅ 标定完成\n");
  fmt::print("   图像尺寸: {}x{}\n", img_size.width, img_size.height);
  fmt::print("   重投影误差: {:.4f} px\n", mean_error);
  fmt::print("   RMS: {:.4f}\n\n", rms);

  // 输出 YAML 格式 (可直接贴到配置文件)
  std::vector<double> cm_data(camera_matrix.ptr<double>(), camera_matrix.ptr<double>() + 9);
  std::vector<double> dc_data(dist_coeffs.ptr<double>(), dist_coeffs.ptr<double>() + dist_coeffs.total());

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Comment(fmt::format("重投影误差: {:.4f}px", mean_error));
  out << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << cm_data;
  out << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow << dc_data;
  out << YAML::EndMap;

  fmt::print("--- 将以下内容复制到 config yaml ---\n");
  fmt::print("{}\n", out.c_str());
  fmt::print("---\n");

  // 保存到文件
  auto out_path = std::filesystem::path(input_folder) / "calibration_result.yaml";
  std::ofstream f(out_path);
  f << "# 相机标定结果\n";
  f << fmt::format("# 来源: {} 张标定图像, 重投影误差: {:.4f}px\n", loaded, mean_error);
  f << fmt::format("# 图像尺寸: {}x{}\n", img_size.width, img_size.height);
  f << out.c_str() << "\n";
  f.close();
  fmt::print("  已保存: {}\n", out_path.string());

  return 0;
}
