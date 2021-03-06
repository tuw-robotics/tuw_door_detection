/**
	* Door detector processing a laser scan. Abstract base class for all detectors.
	*
  * \author felix.koenig@protonmail.com
  * \author felix.koenig@tuwien.ac.at
  */
#ifndef DOOR_DETECTOR_H
#define DOOR_DETECTOR_H

#include <sensor_msgs/LaserScan.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>
#include <laserproc/door_detection.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/publisher.h>
#include <tf/transform_listener.h>

namespace tuw
{

  namespace door_laser_proc
  {

    class DoorDetectorBase
    {

    public:
      std::string DEFAULT_TOPIC_NAME = "door_detections";

      DoorDetectorBase(ros::NodeHandle &_nh, std::string &publisher_topic_name);

      DoorDetectorBase(ros::NodeHandle &_nh);

      virtual ~DoorDetectorBase();

      /** laser scan -> doors */
      virtual bool processLaser(const sensor_msgs::LaserScan &_laser) = 0;

      std::vector<DoorDetectionPtr> &getResult()
      {
        return door_detection_;
      };

      virtual void publish();

      virtual void init(const std::string &pub_topic);

    protected:
      std::vector<tuw_object_msgs::ObjectWithCovariance> objects_;
      ros::Publisher pubObjectDetections_;
      ros::NodeHandle &nh_;
      tf::TransformListener listenerTF_;
      std::unique_ptr<std_msgs::Header> last_header_;
      std::vector<DoorDetectionPtr> door_detection_;

      Eigen::Matrix<double, 4, 4> tf2EigenMat(const tf::Transform &_tf);

      virtual bool getTF(const std::string &world_frame, const std::string &source_frame, tf::StampedTransform &_pose,
                         bool debug = false);

      Eigen::Vector2d range2Eigen(const sensor_msgs::LaserScan &_scan, int idx);

      Eigen::Vector2d range2Eigen(const double range, const double angle);
    };

  };

}

#endif // DOOR_DETECTOR_H
