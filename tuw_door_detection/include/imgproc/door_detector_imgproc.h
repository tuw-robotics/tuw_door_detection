//
// Created by felix on 09.11.18.
//

#ifndef PROJECT_DOOR_DETECTOR_IMGPROC_H
#define PROJECT_DOOR_DETECTOR_IMGPROC_H

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

namespace tuw {
    class DoorDetectorImageProcessor {

    public:
        DoorDetectorImageProcessor();

        ~DoorDetectorImageProcessor();

        void processImage(cv_bridge::CvImagePtr _image_rgb, cv_bridge::CvImagePtr _image_depth = nullptr);

        void display();

        void setStaticImageTF(tf::StampedTransform &tf);

        void setStaticDepthTF(tf::StampedTransform &tf);

    private:
        cv::Mat last_img_processed_;
        cv::Mat last_depth_processed_;
        cv::Mat tfRI;
        cv::Mat tfRD;

    };
};

#endif //PROJECT_DOOR_DETECTOR_IMGPROC_H
