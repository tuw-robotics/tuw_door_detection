/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                         Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include "door_detector_node.h"
#include "laserproc/door_line_detector.h"
#include "laserproc/door_depth_detector.h"

using namespace tuw;

DoorDetectorNode::ParametersNode::ParametersNode() : node("~") {
  std::string mode_str;
  node.param<std::string>("mode", mode_str, std::string("depth"));
  try {
    mode = enumResolver.at(mode_str);
  }
  catch (std::exception &e) {
    ROS_ERROR("ERROR: %s is not a valid detector mode.", mode_str.c_str());
  }
}

DoorDetectorNode::DoorDetectorNode() : nh_(""), display_window_(true), modify_laser_scan_(true) {
  sub_laser_ = nh_.subscribe("scan", 1000, &DoorDetectorNode::callbackLaser, this);
  sub_image_ = nh_.subscribe("image_rgb", 1000, &DoorDetectorNode::callbackImage, this);
  sub_image_depth_ = nh_.subscribe("image_depth", 1000, &DoorDetectorNode::callbackDepthImage, this);
  img_processor_.reset(new DoorDetectorImageProcessor());
  //line_pub_ = nh_.advertise<tuw_geometry_msgs::LineSegments>("line_segments", 1000);
  //door_pub_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("object_detection", 1000);
  //measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();
  params_ = ParametersNode();

  if (params_.mode == ParametersNode::FilterMode::DEPTH) {
    door_detector_.reset(new DoorDepthDetector(nh_));
  } else {
    door_detector_.reset(new DoorLineDetector(nh_));
  }
}

DoorDetectorNode::~DoorDetectorNode() {
}

void DoorDetectorNode::callbackImage(const sensor_msgs::ImageConstPtr &_img) {
  image_rgb_ = cv_bridge::toCvCopy(_img, std::string("8UC3"));

  if (image_rgb_ && image_depth_) {
    img_processor_->processImage(image_rgb_, image_depth_);
    image_rgb_ = nullptr;
    image_depth_ = nullptr;
  }
}

void DoorDetectorNode::callbackDepthImage(const sensor_msgs::ImageConstPtr &_img) {
  image_depth_ = cv_bridge::toCvCopy(_img, std::string("16UC1"));
  image_depth_->image.convertTo(image_depth_->image, CV_32FC1,
                                1.0 / static_cast<double>(std::numeric_limits<u_int16_t>::max()));

  if (image_rgb_ && image_depth_) {
    img_processor_->processImage(image_rgb_, image_depth_);
    image_rgb_ = nullptr;
    image_depth_ = nullptr;
  }
}


void DoorDetectorNode::publish() {
  door_detector_->publish();
  img_processor_->display();
}

void DoorDetectorNode::callbackLaser(const sensor_msgs::LaserScan &_laser) {
  //door_detector_->processLaser(_laser);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "door_2d_detector_node");

  DoorDetectorNode detector_node;

  while (ros::ok()) {
    ros::spinOnce();
    detector_node.publish();

  }
  return 0;
}
