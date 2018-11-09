//
// Created by felix on 09.11.18.
//

#ifndef PROJECT_DOOR_DETECTOR_IMGPROC_NODE_H
#define PROJECT_DOOR_DETECTOR_IMGPROC_NODE_H

#include <door_detector_imgproc.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace tuw {
    class DoorDetectorImageProcessorNode {
    public:
        struct ParametersNode {
            ParametersNode();
        };

        DoorDetectorImageProcessorNode();

        ~DoorDetectorImageProcessorNode();

        //@ToDo: make this shared memory and nodelet
        void callbackImage(const sensor_msgs::Image &img);

    private:
        ros::NodeHandle nh_;
        ParametersNode params_;
        ros::Subscriber sub_image_;
        //ros::Subscriber sub_image_depth_;
        //ros::Subscriber sub_depth_cloud_;

        std::unique_ptr<DoorDetectorImageProcessor> image_processor_;

    };
};
#endif //PROJECT_DOOR_DETECTOR_IMGPROC_NODE_H
