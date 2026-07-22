#include "yolo26n.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

YOLO26N::YOLO26N(const std::string & config_path, bool debug)
: debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);
  model_path_ = yaml["yolo26n_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  if (min_confidence_ < 0.1) min_confidence_ = 0.25;  // 默认不低于0.25

  tools::logger()->info("[YOLO26N] model={}, device={}",
    model_path_, device_);

  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLO26N::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) return {};

  tmp_img_ = raw_img;

  auto x_scale = static_cast<double>(640) / raw_img.rows;
  auto y_scale = static_cast<double>(640) / raw_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(raw_img.rows * scale);
  auto w = static_cast<int>(raw_img.cols * scale);

  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(raw_img, input(roi), {w, h});

  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return postprocess(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLO26N::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // yolo11n ore model output: (1, 5, 8400) [x,y,w,h,conf]
  std::cerr << "D1" << std::endl;
  cv::Mat output_owned = output.clone();
  std::cerr << "D2" << std::endl;
  cv::Mat output_t = output_owned.t();
  std::cerr << "D3 rows=" << output_t.rows << " cols=" << output_t.cols << std::endl;
  
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;

  for (int r = 0; r < output_t.rows; r++) {
    float conf = output_t.at<float>(r, 4);  // single class score
    if (conf < score_threshold_) continue;

    float x = output_t.at<float>(r, 0);
    float y = output_t.at<float>(r, 1);
    float w = output_t.at<float>(r, 2);
    float h = output_t.at<float>(r, 3);

    int left = static_cast<int>((x - 0.5f * w) / scale);
    int top = static_cast<int>((y - 0.5f * h) / scale);
    int width = static_cast<int>(w / scale);
    int height = static_cast<int>(h / scale);

    boxes.emplace_back(left, top, width, height);
    confidences.push_back(conf);
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (int idx : indices) {
    auto & box = boxes[idx];
    std::vector<cv::Point2f> keypoints;
    Armor armor(0, confidences[idx], boxes[idx], keypoints);  // class=0 for ore
    armor.color = Color::red;
    armor.center = cv::Point2f(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
    armor.center_norm = {armor.center.x / bgr_img.cols, armor.center.y / bgr_img.rows};
    armors.push_back(armor);
  }

  if (debug_) {
    auto detection = bgr_img.clone();
    char txt[64];
    for (auto & a : armors) {
      snprintf(txt, sizeof(txt), "%.2f %s", a.confidence, COLORS[a.color]);
      tools::draw_points(detection, a.points, {0, 255, 0});
      tools::draw_text(detection, txt, a.center, {0, 255, 0});
    }
    cv::resize(detection, detection, {}, 0.5, 0.5);
    cv::imshow("detection", detection);
    // waitKey handled by main loop - no duplicate call here
  }

  return armors;
}

void YOLO26N::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  // Handled in postprocess above
}

}  // namespace auto_aim
