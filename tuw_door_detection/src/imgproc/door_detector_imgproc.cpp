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

void DoorDetectorImageProcessor::processImage( std::shared_ptr<ImageMeasurement> &_image_meas_rgb,
                                               std::shared_ptr<ImageMeasurement> &_image_meas_depth ) {
  
  cv::cvtColor( _image_meas_rgb->getImage()->image, _image_meas_rgb->getImage()->image, CV_BGR2GRAY );
  cv::GaussianBlur( _image_meas_rgb->getImage()->image, _image_meas_rgb->getImage()->image, cv::Size( 5, 5 ), 0.75 );
  cv::Canny( _image_meas_rgb->getImage()->image, _image_meas_rgb->getImage()->image, 20, 55, 3, true );
  
  last_img_processed_ = std::move( _image_meas_rgb );
  last_depth_processed_ = std::move( _image_meas_depth );
  
}

void DoorDetectorImageProcessor::registerLaser( std::shared_ptr<LaserMeasurement> &_laser ) {
  
  if ( !last_img_processed_ ) {
    return;
  }
  
  const auto T_CL = last_img_processed_->getTfWorldSensor().inverse() * _laser->getTfWorldSensor();
  
  size_t i = 0;
  laser_img_coords_.resize( _laser->size());
  
  for ( auto beam_it = _laser->begin(); beam_it != _laser->end(); ++beam_it, ++i ) {
    auto endpoint = Eigen::Vector4d( beam_it->end_point.x(), beam_it->end_point.y(), 0, 1 );
    auto laser_in_image = T_CL * endpoint;
    laser_img_coords_[i] = cv::Point2d( endpoint[0], endpoint[1] );
  }
  
}

void DoorDetectorImageProcessor::display() {
  
  if ( last_img_processed_ && last_img_processed_ ) {
    cv::namedWindow( "rgb image processed" );
    cv::imshow( "rgb image processed", last_img_processed_->getImage()->image );
    cv::waitKey( 1 );
  }
  
  if ( last_depth_processed_ && last_depth_processed_ ) {
    cv::namedWindow( "depth image" );
    cv::imshow( "depth image", last_depth_processed_->getImage()->image );
    cv::waitKey( 1 );
  }
  
}