#ifndef STATIC_DOOR_PUBLISHER_NODE
#define STATIC_DOOR_PUBLISHER_NODE

#include <ros/ros.h>

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

  private:
    ros::NodeHandle nh_;
    std::string door_locations_file_;
    std::vector<std::vector<double>> door_positions_;

    void readFile();
    void publishDoors();
  };
};

#endif
