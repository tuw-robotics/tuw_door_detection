#ifndef DUMMY_H
#define DUMMY_H

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectDetection.h>

class Dummy
{
  public:
    Dummy(ros::NodeHandle &n);
    void callbackDoorDetection(const tuw_object_msgs::ObjectDetection &obj_detection);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber door_sub_;
    //ros::Publisher door_pub_;
};

#endif
