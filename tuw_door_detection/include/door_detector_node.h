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

#ifndef LINESEGMENT2D_DETECTOR_NODE_H
#define LINESEGMENT2D_DETECTOR_NODE_H

#include "laserproc/door_detector.h"
#include <imgproc/door_detector_imgproc.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_linedetection/Linesegment2DDetectorConfig.h>
#include <unordered_map>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <map>

namespace tuw {
/**
 * @brief ROS wrapper node for LineSegment2DDetector
 * @class Door2DDetectorNode
 */
    class DoorDetectorNode {
    public:
        struct ParametersNode {
            ParametersNode();

            enum FilterMode {
                LINES, DEPTH
            };
            std::unordered_map<std::string, FilterMode> enumResolver{
                {"lines", FilterMode::LINES},
                {"depth", FilterMode::DEPTH},
                {"LINES", FilterMode::LINES},
                {"DEPTH", FilterMode::DEPTH}
            };
            FilterMode mode;
            ros::NodeHandle node;
            std::string camera_source_frame;
            std::string laser_source_frame;
            std::string world_frame;
            bool debug;
        };

        DoorDetectorNode();

        ~DoorDetectorNode();

        void publish();

    private:
        ros::NodeHandle nh_;
        ParametersNode params_;
        ros::Subscriber sub_laser_;
        ros::Subscriber sub_image_;
        ros::Subscriber sub_image_depth_;
        std::unique_ptr<DoorDetector> door_detector_;
        tf::TransformListener listenerTF_;
        std::map<std::string, std::shared_ptr<tf::StampedTransform>> tfMap_;

        cv_bridge::CvImagePtr image_rgb_;
        cv_bridge::CvImagePtr image_depth_;
        std::unique_ptr<DoorDetectorImageProcessor> img_processor_;

        bool display_window_;
        bool modify_laser_scan_;

        /**
         * @brief callback function for incoming laser scans
         * @param _laser laser scan message
         */
        void callbackLaser(const sensor_msgs::LaserScan &_laser);

        void callbackImage(const sensor_msgs::ImageConstPtr &_img);

        void callbackDepthImage(const sensor_msgs::ImageConstPtr &_img);

        bool getStaticTF(const std::string &world_frame, const std::string &source_frame, tf::StampedTransform &_pose,
                         bool debug);
    };
};

#endif  // LINESEGMENT2D_DETECTOR_NODE_H
