//
// Created by felix on 09.11.18.
//

#ifndef PROJECT_DOOR_DETECTOR_IMGPROC_H
#define PROJECT_DOOR_DETECTOR_IMGPROC_H

#include <cv_bridge/cv_bridge.h>

namespace tuw {
    class DoorDetectorImageProcessor {

    public:
        DoorDetectorImageProcessor();

        ~DoorDetectorImageProcessor();

        void processImage(cv_bridge::CvImagePtr _image_rgb, cv_bridge::CvImagePtr _image_depth = nullptr);

        void display();

    private:
        cv::Mat last_img_processed_;
        cv::Mat last_depth_processed_;

    };
};

#endif //PROJECT_DOOR_DETECTOR_IMGPROC_H
