#include <imgproc/door_detector_imgproc.h>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <tuw_geometry/utils.h>

//
// Created by felix on 09.11.18.
//

using namespace tuw;

DoorDetectorImageProcessor::DoorDetectorImageProcessor() {

}

DoorDetectorImageProcessor::~DoorDetectorImageProcessor() {

}

void DoorDetectorImageProcessor::processImage(std::unique_ptr<ImageMeasurement> &_image_meas_rgb, std::unique_ptr<ImageMeasurement> &_image_meas_depth) {
  cv::cvtColor(_image_meas_rgb->getImage()->image, _image_meas_rgb->getImage()->image, CV_BGR2GRAY);
  cv::GaussianBlur(_image_meas_rgb->getImage()->image, _image_meas_rgb->getImage()->image, cv::Size(5, 5), 0.75);
  cv::Canny(_image_meas_rgb->getImage()->image, _image_meas_rgb->getImage()->image, 20, 55, 3, true);

  last_img_processed_ = _image_meas_rgb->getImage()->image;
  last_depth_processed_ = _image_meas_depth->getImage()->image;
}

void DoorDetectorImageProcessor::display() {
  if (last_img_processed_.rows > 0 && last_img_processed_.cols > 0) {
    cv::namedWindow("rgb image processed");
    cv::imshow("rgb image processed", last_img_processed_);
    cv::waitKey(1);
  }

  if (last_depth_processed_.rows > 0 && last_depth_processed_.cols > 0) {
    cv::namedWindow("depth image");
    cv::imshow("depth image", last_depth_processed_);
    cv::waitKey(1);
  }
}