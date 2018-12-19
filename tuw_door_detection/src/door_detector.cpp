//
// Created by felix on 14.12.18.
//

#include <door_detector.h>
#include <memory>
#include <opencv2/highgui.hpp>

using namespace tuw;

DoorDetector::DoorDetector()
{

}

DoorDetector::~DoorDetector()
{

}

void DoorDetector::merge( std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
                          std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor )
{
  
  detection_image_ = img_processor->getResult();
  auto laser_processor_dyn = std::dynamic_pointer_cast<door_laser_proc::DoorDepthDetector>(
      laser_processor );
  
  if ( !laser_processor_dyn )
  {
    return;
  }
  
  detection_laser_ = laser_processor_dyn->getContours();
  
  door_candidates_ = std::vector<std::shared_ptr<Contour>>( detection_laser_.size());
  auto it = std::copy_if( detection_laser_.begin(),
                          detection_laser_.end(),
                          door_candidates_.begin(),
                          []( const std::shared_ptr<Contour> c )
                          {
                            return c->is_door_candidate();
                          } );
  
  door_candidates_.resize( std::distance( door_candidates_.begin(), it ));
  
}

void DoorDetector::clear()
{
  image_measurement_ = nullptr;
  laser_measurement_ = nullptr;
}

void DoorDetector::setImageMeasurement( std::shared_ptr<ImageMeasurement> &img_meas )
{
  image_measurement_ = img_meas;
}

void DoorDetector::setLaserMeasurement( std::shared_ptr<tuw::LaserMeasurement> &laser_meas )
{
  laser_measurement_ = laser_meas;
}

void DoorDetector::display()
{
  
  if ( image_measurement_ && laser_measurement_ )
  {
    cv::Mat img_display;
    image_measurement_->getOriginalImage().copyTo( img_display );
    if ( img_display.channels() != 3 )
    {
      cv::cvtColor( img_display, img_display, CV_GRAY2BGR );
    }
    
    const auto T_WC = image_measurement_->getTfWorldSensor();
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
        const cv::Point3d pnt3d = cv::Point3d( laser_in_image[0],
                                               laser_in_image[1],
                                               laser_in_image[2] );
        
        const cv::Point2d img_pnt = image_measurement_->getCameraModel()->project3dToPixel( pnt3d );
        
        //@ToDo: do this independently in separate method -> image coordinates are needed more than once
        beam->img_coords.x() = img_pnt.x;
        beam->img_coords.y() = img_pnt.y;
        
        //in image coord
        cv::circle( img_display, img_pnt, rad, contour->getAssignedColor(), rad );
        
      }
      
    }
    
    //std::cout << "door candidates size " << door_candidates_.size() << std::endl << std::endl;
    //
    for ( auto it_contour = door_candidates_.begin();
          it_contour != door_candidates_.end();
          ++it_contour )
    {
      std::shared_ptr<Contour> contour = *it_contour;
      std::cout << "c " << contour->beams().size() << std::endl << std::endl;
      Point2D right_most = contour->front()->img_coords;
      std::cout << "c" << std::endl << std::endl;
      Point2D left_most = contour->back()->img_coords;
      std::cout << "c" << std::endl << std::endl;
      // cv::Rect roi( left_most.y(), 0, right_most.y() - left_most.y(), img_display.size().height );
      // cv::rectangle( img_display, roi, cv::Scalar( 0, 255, 0 ), 1 );
    }
    
    cv::imshow( "rgb image processed", img_display );
    cv::waitKey( 100 );
    
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
