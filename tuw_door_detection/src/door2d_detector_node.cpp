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

#include "door2d_detector_node.h"

#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Core>

using namespace tuw;

Door2DDetectorNode::Door2DDetectorNode() : nh_private_("~"), display_window_(true), modify_laser_scan_(true), doors_filter_mode_(Door2DDetectorNode::FilterMode::FILTER_DOORS)
{
  sub_laser_ = nh_.subscribe("scan", 1000, &Door2DDetectorNode::callbackLaser, this);
  line_pub_ = nh_.advertise<tuw_geometry_msgs::LineSegments>("line_segments", 1000);
  door_pub_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("object_detection", 1000);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/r0/laser0/scan/filtered", 1000);
  measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();
}

void Door2DDetectorNode::callbackLaser(const sensor_msgs::LaserScan& _laser)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "linesegment2d_detector_node");

  Door2DDetectorNode detector_node;

  ros::spin();

  return 0;
}
