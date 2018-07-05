#ifndef STATIC_DOOR_PUBLISHER_NODE
#define STATIC_DOOR_PUBLISHER_NODE

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <eigen3/Eigen/Core>
#include <base_pub_object.h>

namespace tuw
{
/**
 * @brief ROS wrapper node for LineSegment2DDetector
 * @class Door2DDetectorNode
 */
  class ObjectPublisher
  {
  public:
    ObjectPublisher(ros::NodeHandle &n);
    void init();
    void publish();
//    void rotate(int i, double rad);
//    void add_position(int i, Eigen::Vector3d position);

  private:
    ros::NodeHandle nh_;
    ros::Publisher door_pub_;
    std::string door_locations_file_;
    std::vector<std::unique_ptr<tuw::BasePubObject>> objects_;
    unsigned int nr_objects;
  };
};

#endif
