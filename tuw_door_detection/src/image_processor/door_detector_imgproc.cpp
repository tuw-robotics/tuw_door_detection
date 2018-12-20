#include <imgproc/door_detector_imgproc.h>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <tuw_geometry/utils.h>

//
// Created by felix on 09.11.18.
//

using namespace tuw;
using namespace image_processor;

DoorDetectorImageProcessor::DoorDetectorImageProcessor() : last_door_detection_(std::make_shared<DoorDetection>())
{
}

DoorDetectorImageProcessor::~DoorDetectorImageProcessor() = default;

void DoorDetectorImageProcessor::processImage(std::shared_ptr<ImageMeasurement> &_image_meas_rgb,
                                              std::shared_ptr<ImageMeasurement> &_image_meas_depth)
{

  last_img_processed_ = _image_meas_rgb;
  last_depth_processed_ = _image_meas_depth;

  cv::Mat &img_cv = _image_meas_rgb->cv();
  cv::GaussianBlur(img_cv, img_cv, cv::Size(5, 5), 0.75);
  cv::Canny(img_cv, img_cv, 20, 55, 3, true);

  last_door_detection_->setImageMeasurement(_image_meas_rgb);
}

std::shared_ptr<DoorDetection> &DoorDetectorImageProcessor::getResult()
{
  return last_door_detection_;
}
