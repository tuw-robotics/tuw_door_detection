//
// Created by felix on 14.12.18.
//

#ifndef IMGPROC_DOOR_DETECTION_H
#define IMGPROC_DOOR_DETECTION_H

#include <tuw_measurement_utils/image_measurement.h>

namespace tuw
{
  namespace image_processor
  {

    class DoorDetection;

    using DoorDetectionPtr = std::shared_ptr<DoorDetection>;
    using DoorDetectionConstPtr = std::shared_ptr<DoorDetection const>;

    class DoorDetection
    {
    public:

      void setImageMeasurement( const std::shared_ptr<ImageMeasurement> &_image )
      {
        image_ = _image;
      };

      const std::shared_ptr<ImageMeasurement> &getImageMeasurement() const
      {
        return image_;
      }

    private:

      std::shared_ptr<ImageMeasurement> image_;

    };

  };
}

#endif //PROJECT_DOOR_DETECTION_H
