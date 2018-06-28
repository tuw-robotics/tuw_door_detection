#ifndef STATIC_DOOR_PUBLISHER_NODE
#define STATIC_DOOR_PUBLISHER_NODE

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectDetection.h>

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
    void callbackObjectDetection(tuw_object_msgs::ObjectDetection &obj_detection);

  private:
    ros::NodeHandle nh_;
    ros::Publisher door_pub_;
    std::string door_locations_file_;
    std::vector<std::vector<double>> door_positions_;
    tuw_object_msgs::ObjectDetection door_detection_msg_;

    void readFile();
    void prepareMsgs();
  };
};

#endif
