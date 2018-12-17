#include <imgproc/door_detector_imgproc.h>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <tuw_geometry/utils.h>

//
// Created by felix on 09.11.18.
//

using namespace tuw;
using namespace image_processor;

DoorDetectorImageProcessor::DoorDetectorImageProcessor() : last_door_detection_( std::make_shared<DoorDetection>())
{
}

DoorDetectorImageProcessor::~DoorDetectorImageProcessor() = default;

void DoorDetectorImageProcessor::processImage( std::shared_ptr<ImageMeasurement> &_image_meas_rgb,
                                               std::shared_ptr<ImageMeasurement> &_image_meas_depth )
{
  
  last_img_processed_ = _image_meas_rgb;
  last_depth_processed_ = _image_meas_depth;
  
  cv::Mat &img_cv = _image_meas_rgb->cv();
  cv::cvtColor( img_cv, img_cv, CV_BGR2GRAY );
  cv::GaussianBlur( img_cv, img_cv, cv::Size( 5, 5 ), 0.75 );
  cv::Canny( img_cv, img_cv, 20, 55, 3, true );
  
  last_door_detection_->setImageMeasurement( _image_meas_rgb );
}

void DoorDetectorImageProcessor::registerLaser( std::shared_ptr<LaserMeasurement> &_laser )
{
  
  if ( !last_img_processed_ )
  {
    return;
  }
  
  laser_img_coords_.clear();
  
  const auto T_WC = last_img_processed_->getTfWorldSensor();
  const auto T_WL = _laser->getTfWorldSensor();
  const auto T_CL = T_WC.inverse() * T_WL;
  
  size_t i = 0;
  laser_img_coords_.resize( _laser->size());
  
  for ( auto beam_it = _laser->begin(); beam_it != _laser->end(); ++beam_it, ++i )
  {
    auto endpoint = Eigen::Vector4d( beam_it->end_point.x(), beam_it->end_point.y(), 0, 1 );
    Eigen::Vector4d laser_in_image = T_CL * endpoint;
    laser_in_image = laser_in_image / laser_in_image[3];
    
    //const auto pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
    const cv::Point3d pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
    laser_img_coords_[i] = last_img_processed_->getCameraModel()->project3dToPixel( pnt3d );
  }
  
  //laser_img_coords_.resize( 1000 );
  //double _x = 0.0;
  //double _y = 0.0;
  //for ( int i = 0; i < 1000; ++i ) {
  //  laser_img_coords_.push_back( last_img_processed_->getCameraModel()->project3dToPixel(
  //      cv::Point3d( _x + 0.001 * i, _y, 1.0 )));
  //  //laser_img_coords_.push_back( last_img_processed_->getCameraModel()->project3dToPixel(
  //  //cv::Point3d( _x, _y + 0.001 * i, 1.0 )));
  //}
}

void DoorDetectorImageProcessor::display()
{
  
  //@ToDo: performance
  if ( last_img_processed_ )
  {
    cv::Mat img_display;
    last_door_detection_->getImageMeasurement()->cv().copyTo( img_display );
    if ( img_display.channels() != 3 )
    {
      cv::cvtColor( img_display, img_display, CV_GRAY2BGR );
    }
    cv::namedWindow( "rgb image processed" );
    std::size_t idx_start = 0;
    std::size_t idx_end = laser_img_coords_.size() - 1;
    std::size_t counter = 0;
    
    for ( cv::Point2d &pt : laser_img_coords_ )
    {
      double rad = 2;
      if ( counter == idx_start || counter == idx_end )
      {
        rad = 5;
      }
      
      //auto pt_switched = cv::Point2d( pt.y, pt.x );
      //std::cout << pt << std::endl;
      cv::circle( img_display, pt/*pt_switched*/, rad, , rad );
      counter++;
    }
    
    cv::imshow( "rgb image processed", img_display );
    cv::waitKey( 1 );
    
  }
  
  if ( last_depth_processed_ )
  {
    
    cv::namedWindow( "depth image" );
    cv::imshow( "depth image", last_depth_processed_->cv());
    cv::waitKey( 1 );
    
  }
  
}

std::shared_ptr<DoorDetection> &DoorDetectorImageProcessor::getResult()
{
  return last_door_detection_;
}
