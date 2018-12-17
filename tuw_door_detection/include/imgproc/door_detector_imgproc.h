//
// Created by felix on 09.11.18.
//

#ifndef PROJECT_DOOR_DETECTOR_IMGPROC_H
#define PROJECT_DOOR_DETECTOR_IMGPROC_H

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_measurement_utils/measurements.h>
#include <imgproc/door_detection.h>

namespace tuw
{
  namespace image_processor
  {
    
    class DoorDetectorImageProcessor
    {
    
    public:
      DoorDetectorImageProcessor();
      
      ~DoorDetectorImageProcessor();
      
      void processImage( std::shared_ptr<ImageMeasurement> &_image_meas_rgb,
                         std::shared_ptr<ImageMeasurement> &_image_meas_depth );
      
      std::shared_ptr<DoorDetection> &getResult();
      
      void registerLaser( std::shared_ptr<LaserMeasurement> &_laser );
      
      void display();
    
    private:
      std::shared_ptr<ImageMeasurement> last_img_processed_;
      std::shared_ptr<ImageMeasurement> last_depth_processed_;
      std::shared_ptr<DoorDetection> last_door_detection_;
      cv::Mat tfRI;
      cv::Mat tfRD;
      std::vector<cv::Point2d> laser_img_coords_;
      std::vector<cv::Point2d> corner_img_coords_;
      
    };
    
  };
  
};

#endif //PROJECT_DOOR_DETECTOR_IMGPROC_H
