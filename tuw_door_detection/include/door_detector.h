//
// Created by felix on 14.12.18.
//

#ifndef TUW_DOOR_DETECTOR_H
#define TUW_DOOR_DETECTOR_H

#include <imgproc/door_detector_imgproc.h>
#include <laserproc/door_depth_detector.h>

namespace tuw
{
  
  class DoorDetector
  {
  public:
    
    DoorDetector();
    
    ~DoorDetector();
    
    void merge( std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
                std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor );
    
    void setImageMeasurement( std::shared_ptr<ImageMeasurement> &image_meas );
    
    void setLaserMeasurement( std::shared_ptr<LaserMeasurement> &laser_meas );
    
    void clear();
    
    void display();
  
  private:
    std::shared_ptr<ImageMeasurement> image_measurement_;
    std::shared_ptr<LaserMeasurement> laser_measurement_;
    image_processor::DoorDetectionPtr detection_image_;
    std::vector<std::shared_ptr<Contour>> detection_laser_;
    std::vector<std::shared_ptr<Contour>> door_candidates_;
  };
  
};

#endif //PROJECT_DOOR_DETECTOR_H
