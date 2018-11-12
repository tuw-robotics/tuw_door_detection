#include <imgproc/door_detector_imgproc.h>
#include <opencv2/highgui.hpp>

//
// Created by felix on 09.11.18.
//

using namespace tuw;

DoorDetectorImageProcessor::DoorDetectorImageProcessor() {

}

DoorDetectorImageProcessor::~DoorDetectorImageProcessor() {

}

void DoorDetectorImageProcessor::processImage(cv_bridge::CvImagePtr _image_rgb, cv_bridge::CvImagePtr _image_depth) {
  cv::cvtColor(_image_rgb->image, _image_rgb->image, CV_BGR2GRAY);
  //std::vector<cv::Mat> denoised;
  //denoised.push_back(_image_rgb->image);
  //cv::denoise_TVL1(denoised, _image_rgb->image);
  cv::GaussianBlur(_image_rgb->image, _image_rgb->image, cv::Size(5, 5), 0.75);
  cv::Canny(_image_rgb->image, _image_rgb->image, 20, 55, 3, true);

  last_img_processed_ = _image_rgb->image;
  last_depth_processed_ = _image_depth->image;
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