//
// Created by felix on 09.11.18.
//

#include <imgproc/door_detector_imgproc_node.h>
#include <opencv2/highgui.hpp>

using namespace tuw;

DoorDetectorImageProcessorNode::ParametersNode::ParametersNode() {

}

DoorDetectorImageProcessorNode::DoorDetectorImageProcessorNode() : nh_("") {
    sub_image_ = nh_.subscribe("image_rgb", 1000, &DoorDetectorImageProcessorNode::callbackImage, this);
    //sub_image_depth_ = nh_.subscribe("depth_image", 1000, &DoorDetectorImageProcessorNode::callbackDepthImage, this);
    //sub_image_depth_ = nh_.subscribe("depth", 1000, &DoorDetectorImageProcessorNode::callbackImage, this);
}

DoorDetectorImageProcessorNode::~DoorDetectorImageProcessorNode() {

}

void DoorDetectorImageProcessorNode::callbackImage(const sensor_msgs::ImageConstPtr &_img) {
    auto img = cv_bridge::toCvCopy(_img, std::string("CV_8UC3"));
    image_rgb_ = img;
    std::cout << "image gotten" << std::endl;
}

void DoorDetectorImageProcessorNode::callbackDepthImage(const sensor_msgs::ImageConstPtr &_img) {
    auto img = cv_bridge::toCvCopy(_img, std::string("CV_8UC3"));
    image_depth_ = img;
}

void DoorDetectorImageProcessorNode::display() {
    std::cout << "display" << std::endl;
    if (image_rgb_)
    {
        cv::namedWindow("rgb image");
        cv::imshow("rgb image", image_rgb_->image);
        cv::waitKey(1);
    }

    if (image_depth_)
    {
        cv::namedWindow("depth image");
        cv::imshow("depth image", image_depth_->image);
        cv::waitKey(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "door_detector_image_processor_node");
    DoorDetectorImageProcessorNode node;

    while (ros::ok()) {
        ros::spinOnce();
        node.display();
    }
}