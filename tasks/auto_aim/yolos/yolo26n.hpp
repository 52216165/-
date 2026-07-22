#ifndef AUTO_AIM__YOLO26N_HPP
#define AUTO_AIM__YOLO26N_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class YOLO26N : public YOLOBase
{
public:
  YOLO26N(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string device_, model_path_;
  bool debug_, use_roi_;

  const int num_classes_ = 1; // ore
  const float nms_threshold_ = 0.4f;
  const float score_threshold_ = 0.01f;  // 模型置信度极低，尽量接受
  double min_confidence_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO26N_HPP
