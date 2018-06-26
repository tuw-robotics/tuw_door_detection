#include "static_door_publisher_node.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "linesegment2d_detector_node");

  Door2DDetectorNode detector_node;

  ros::spin();

  return 0;
}
