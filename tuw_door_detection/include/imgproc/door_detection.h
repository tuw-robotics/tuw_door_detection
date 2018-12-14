//
// Created by felix on 14.12.18.
//

#ifndef IMGPROC_DOOR_DETECTION_H
#define IMGPROC_DOOR_DETECTION_H

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

      void setImage(const cv::Mat &_image)
      { image_(_image); };

      const cv::Mat &getImage()
      { return image_; }

    private:

      cv::Mat image_;

    };

  };
}

#endif //PROJECT_DOOR_DETECTION_H
