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
  
  //for ( const std::shared_ptr<Contour> &c : detection_laser_ )
  //{
  //  if ( c->is_door_candidate())
  //  {
  //    door_candidates_.push_back( c );
  //  }
  //  for ( auto &child_candidate : c->getChildren())
  //  {
  //    if ( child_candidate->is_door_candidate())
  //    {
  //      door_candidates_.push_back( child_candidate );
  //    }
  //  }
  //}
  
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
    const Eigen::Matrix4d T_CL = T_WC.inverse() * T_WL;
    
    auto &cmodel = image_measurement_->getCameraModel();
    auto img_height = img_display.size().height;
    auto img_width = img_display.size().width;
    
    std::for_each( detection_laser_.begin(), detection_laser_.end(),
                   [&cmodel, &T_CL, img_height, img_width]
                       ( std::shared_ptr<Contour> &contr )
                   {
                     contr->registerToImage( T_CL,
                                             cmodel->fx(), cmodel->fy(),
                                             cmodel->cx(), cmodel->cy(),
                                             cmodel->Tx(), cmodel->Ty());
      
                     contr->visibilityCheck( false, img_width, img_height );
                   } );
    
    cv::namedWindow( "rgb image processed" );
    for ( const auto &contour : detection_laser_ )
    {
      contour->registerToImage( T_CL,
                                cmodel->fx(), cmodel->fy(),
                                cmodel->cx(), cmodel->cy(),
                                cmodel->Tx(), cmodel->Ty());
      
      contour->visibilityCheck( false, img_width, img_height );
      
      for ( auto it_beams = contour->begin();
            it_beams != contour->end(); ++it_beams )
      {
        std::shared_ptr<Contour::Beam> &beam = *it_beams;
        
        double rad = 1;
        if ( it_beams == contour->begin() || it_beams == (contour->end() - 1))
        {
          rad = 2;
        }
        
        if ( beam->get_is_visible())
        {
          auto &img_pnt = beam->img_coords;
          cv::circle( img_display, img_pnt.cv(),
                      rad, contour->getAssignedColor(), rad );
        }
      }
      
      for ( const auto &line_seg : contour->getLineSegmentImageCoords())
      {
        cv::Point2d p0 = line_seg.first.cv();
        cv::Point2d p1 = line_seg.second.cv();
        cv::line( img_display, p0, p1, cv::Scalar( 0, 0, 0 ));
      }
    }
    
    //std::cout << "door candidates size " << door_candidates_.size() << std::endl << std::endl;
    //
    for ( auto it_contour = detection_laser_.begin();
          it_contour != detection_laser_.end();
          ++it_contour )
    {
      std::shared_ptr<Contour> contour = *it_contour;
      if ( contour->is_door_candidate())
      {
        draw_roi( contour, img_display );
      }
      for ( auto chld: contour->getChildren())
      {
        if ( chld->is_door_candidate())
        {
          std::cout << "child is door candidate" << std::endl;
          draw_roi( chld, img_display );
        }
      }
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

void DoorDetector::draw_roi( std::shared_ptr<Contour> &contour, cv::Mat &img_display )
{
  Point2D right_most, left_most;
  if ( contour->beams().front()->get_is_visible() && contour->beams().back()->get_is_visible())
  {
    right_most = contour->beams().front()->img_coords;
    left_most = contour->beams().back()->img_coords;
    printf( "left (%.2f,%.2f)\n", left_most.x(), left_most.y());
    printf( "right (%.2f,%.2f)\n", right_most.x(), right_most.y());
    cv::Rect roi( left_most.x(), 0, right_most.x() - left_most.x(), img_display.size().height );
    cv::rectangle( img_display, roi, cv::Scalar( 0, 255, 0 ), 1 );
  }
}
