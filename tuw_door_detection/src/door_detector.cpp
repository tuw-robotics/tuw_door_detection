//
// Created by felix on 14.12.18.
//

#include <door_detector.h>
#include <memory>

using namespace tuw;

void DoorDetector::merge(std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
                         std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor)
{
  image_processor::DoorDetectionPtr detection_image = img_processor->getResult();
  std::vector<std::shared_ptr<tuw::Contour>> detection_laser = std::dynamic_pointer_cast<door_laser_proc::DoorDepthDetector>(
      laser_processor)->getContours();

  if (!detection_image || !detection_laser.size())
  {
    return;
  }
}
