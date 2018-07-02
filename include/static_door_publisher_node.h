#ifndef STATIC_DOOR_PUBLISHER_NODE
#define STATIC_DOOR_PUBLISHER_NODE

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <eigen3/Eigen/Core>

namespace tuw
{
/**
 * @brief ROS wrapper node for LineSegment2DDetector
 * @class Door2DDetectorNode
 */
  class StaticDoorPublisherNode
  {
  public:
    StaticDoorPublisherNode(ros::NodeHandle &n);
    void init();
    void publishDoors();
    void rotate(int i, double rad);
    void add_position(int i, Eigen::Vector3d position);

  private:
    ros::NodeHandle nh_;
    ros::Publisher door_pub_;
    std::string door_locations_file_;
    std::vector<std::vector<double>> door_positions_;
    tuw_object_msgs::ObjectDetection door_detection_msg_;

    void readFile();
    void prepareMsgs();
    Eigen::Matrix3d rotation_matrix_z(double rad);
    double deg2rad(int degrees);
  };
};

#endif
