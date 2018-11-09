//
// Created by felix on 09.11.18.
//

#include "door_detector_imgproc_node.h"

using namespace tuw;

DoorDetectorImageProcessorNode::ParametersNode::ParametersNode() {

}

DoorDetectorImageProcessorNode::DoorDetectorImageProcessorNode() : nh_("") {
    sub_image_ = nh_.subscribe("image", 1000, &DoorDetectorImageProcessorNode::callbackImage, this);
    //sub_image_depth_ = nh_.subscribe("depth", 1000, &DoorDetectorImageProcessorNode::callbackImage, this);
}

DoorDetectorImageProcessorNode::~DoorDetectorImageProcessorNode() {

}

void DoorDetectorImageProcessorNode::callbackImage(const sensor_msgs::Image &img) {
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "door_detector_image_processor_node");
    DoorDetectorImageProcessorNode node;

    while (ros::ok()) {
        ros::spinOnce();
    }
}