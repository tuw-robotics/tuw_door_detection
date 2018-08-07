#ifndef BASE_PUB_OBJECT_H
#define BASE_PUB_OBJECT_H

#include <string>
#include <vector>
#include <tuw_object_msgs/ObjectDetection.h>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ostream>

namespace tuw {
  class BasePubObject {
    public:
      BasePubObject(std::string &type, std::string &file_path, std::string &publisher_topic);
      virtual ~BasePubObject();

      virtual void setPublisher(ros::NodeHandle &nh);
      virtual void publish();

      virtual bool read();
      virtual bool createMsg() = 0;

      bool multiply_tf(tf::StampedTransform &_tf, bool _keep_old);
      bool restore_tf();
      void tf2pose(tf::Transform &_tf, geometry_msgs::Pose &p);
      void pose2tf(geometry_msgs::Pose &p, tf::Transform &_tf);
      void set_pose_from_tf(tf::StampedTransform &_tf);

      const std::string as_string();

      const tuw_object_msgs::ObjectDetection &getMsg() const { return msg_; }
      tuw_object_msgs::ObjectDetection &getMsg() { return msg_; }

    protected:
      std::string file_path_;
      std::string type_;
      std::string publisher_topic_name_;
      ros::Publisher pub_;
      std::vector<std::vector<std::string>> file_contents_;
      tuw_object_msgs::ObjectDetection msg_;
      std::vector<geometry_msgs::Pose> old_tfs_;

      double deg2rad(int degrees);
  };
}

#endif
