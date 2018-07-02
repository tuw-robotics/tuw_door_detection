#include "dummy.h"

Dummy::Dummy(ros::NodeHandle &n)
{
  nh_ = n;
  door_sub_ = nh_.subscribe("doors_detected", 1000, &Dummy::callbackDoorDetection, this);
}

void Dummy::callbackDoorDetection(const tuw_object_msgs::ObjectDetection &obj_detection)
{
  for (tuw_object_msgs::ObjectWithCovariance obj : obj_detection.objects)
  {
    std::cout << "pose x " << obj.object.pose.position.x << ", y " << obj.object.pose.position.y << ", z " << obj.object.pose.position.z << std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_node");
  ros::NodeHandle n;
  Dummy dummy_node(n);

  ros::spin();

  return 0;
}
