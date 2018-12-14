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
    void merge(std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
               std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor);
  };

};

#endif //PROJECT_DOOR_DETECTOR_H
