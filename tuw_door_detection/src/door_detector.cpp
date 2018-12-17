//
// Created by felix on 14.12.18.
//

#include <door_detector.h>
#include <memory>
#include <opencv2/highgui.hpp>

using namespace tuw;

void DoorDetector::merge( std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
                          std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor )
{
  
  detection_image_ = img_processor->getResult();
  auto laser_processor_dyn = std::dynamic_pointer_cast<door_laser_proc::DoorDepthDetector>(
      laser_processor );
  if (laser_processor_dyn)
  {
    detection_laser_ = laser_processor_dyn->getContours();
  }
  
  if ( !detection_image_ || !detection_laser_.size())
  {
    return;
  }
  
}

void DoorDetector::setImageMeasurement(std::shared_ptr<ImageMeasurement> &img_meas)
{
  image_measurement_ = img_meas;
}

void DoorDetector::setLaserMeasurement( std::shared_ptr<tuw::LaserMeasurement> &laser_meas )
{
  laser_measurement_ = laser_meas;
}

void DoorDetector::display()
{
  
  if ( detection_image_ && detection_laser_.size())
  {
    cv::Mat img_display;
    detection_image_->getImageMeasurement()->cv().copyTo( img_display );
    if ( img_display.channels() != 3 )
    {
      cv::cvtColor( img_display, img_display, CV_GRAY2BGR );
    }
    
    const auto T_WC = detection_image_->getImageMeasurement()->getTfWorldSensor();
    const auto T_WL = laser_measurement_->getTfWorldSensor();
    const auto T_CL = T_WC.inverse() * T_WL;
  
    cv::namedWindow( "rgb image processed" );
    for ( const auto &contour : detection_laser_ )
    {
      
      for ( auto it_beams = contour->begin();
            it_beams != contour->end(); ++it_beams )
      {
        auto beam = *it_beams;
        
        double rad = 2;
        if ( it_beams == contour->begin() || it_beams == (contour->end() - 1))
        {
          rad = 5;
        }
        
        auto endpoint = Eigen::Vector4d( beam->end_point.x(), beam->end_point.y(), 0, 1 );
        Eigen::Vector4d laser_in_image = T_CL * endpoint;
        laser_in_image = laser_in_image / laser_in_image[3];
        
        //const auto pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
        const cv::Point3d pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
        const cv::Point2d img = detection_image_->getImageMeasurement()->getCameraModel()->project3dToPixel( pnt3d );
        
        //in image coord
        cv::circle( img_display, img, rad, contour->getAssignedColor(), rad );
      }
    }
    
    cv::imshow( "rgb image processed", img_display );
    cv::waitKey( 1 );
    
  }
  
  for ( const auto c : detection_laser_ )
  {
    //auto corners = c->getCorners();
    //std::cout << "c " << corners.size() << std::endl;
    //for ( const std::unique_ptr<Contour::Corner> &cn : corners )
    //{
    //}
  }
}
