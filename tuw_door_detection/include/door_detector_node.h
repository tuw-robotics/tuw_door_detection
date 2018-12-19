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

#include <laserproc/door_depth_detector.h>
#include <imgproc/door_detector_imgproc.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <tuw_measurement_utils/measurements.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <door_detector.h>

namespace tuw
{
/**
 * @brief ROS wrapper node for LineSegment2DDetector
 * @class Door2DDetectorNode
 */
  class DoorDetectorNode
  {
  public:
    struct ParametersNode
    {
      ParametersNode();

      enum FilterMode
      {
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

    void process();

    void publish();

  private:
    ros::NodeHandle nh_;
    ParametersNode params_;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_image_;
    ros::Subscriber sub_camera_info_rgb_;
    ros::Subscriber sub_image_depth_;
    ros::Subscriber sub_camera_info_depth_;
    std::shared_ptr<door_laser_proc::DoorDetectorBase> door_detector_laser_;
    std::unique_ptr<DoorDetector> door_detector_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::map<std::string, geometry_msgs::TransformStampedPtr> tfMap_;

    std::shared_ptr<ImageMeasurement> image_depth_;
    std::shared_ptr<ImageMeasurement> image_rgb_;
    std::shared_ptr<LaserMeasurement> laser_measurement_;
    std::shared_ptr<image_processor::DoorDetectorImageProcessor> img_processor_;
    sensor_msgs::CameraInfoPtr camera_info_depth_;
    sensor_msgs::CameraInfoPtr camera_info_rgb_;
    //std::unique_ptr<DoorDepthDetector> depth_detector_;

    bool display_window_;
    bool modify_laser_scan_;

    /**
     * @brief callback function for incoming laser scans
     * @param _laser laser scan message
     */
    void callbackLaser(const sensor_msgs::LaserScan &_laser);

    void callbackCameraInfoRGB(const sensor_msgs::CameraInfoConstPtr &_msg);

    void callbackCameraInfoDepth(const sensor_msgs::CameraInfoConstPtr &_msg);

    void callbackImage(const sensor_msgs::ImageConstPtr &_img);

    void callbackDepthImage(const sensor_msgs::ImageConstPtr &_img);

    bool getStaticTF(const std::string &world_frame, const std::string &source_frame,
                     geometry_msgs::TransformStampedPtr &_pose,
                     bool debug);
  };
};

#endif  // LINESEGMENT2D_DETECTOR_NODE_H
