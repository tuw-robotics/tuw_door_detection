//
// Created by felix on 09.11.18.
//

#ifndef PROJECT_DOOR_DETECTOR_IMGPROC_H
#define PROJECT_DOOR_DETECTOR_IMGPROC_H

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <measurements.h>

namespace tuw {
    class DoorDetectorImageProcessor {

    public:
        DoorDetectorImageProcessor();

        ~DoorDetectorImageProcessor();

        void processImage(std::unique_ptr<ImageMeasurement> &_image_meas_rgb,
                          std::unique_ptr<ImageMeasurement> &_image_meas_depth);

        void display();

    private:
        cv::Mat last_img_processed_;
        cv::Mat last_depth_processed_;
        cv::Mat tfRI;
        cv::Mat tfRD;

    };
};

#endif //PROJECT_DOOR_DETECTOR_IMGPROC_H
