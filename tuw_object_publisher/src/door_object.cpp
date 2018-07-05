#include "door_object.h"
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

using namespace tuw;

DoorObject::DoorObject(std::string &type,std::string &file_path, std::string &publisher_topic) : BasePubObject(type, file_path, publisher_topic)
{}

DoorObject::~DoorObject(){}

bool DoorObject::createMsg()
{
  for (auto line : file_contents_)
  {
    std::vector<double> v(line.size());
    for (int j=0; j < v.size(); j++)
      v[j] = std::stod(line[j]);
    file_contents_parsed_.push_back(v);
  }

  msg_ = tuw_object_msgs::ObjectDetection();
  msg_.header.stamp = ros::Time::now();
  msg_.header.frame_id = std::string("map");
  msg_.type = std::string("");
  int count = 0;
  std::for_each(file_contents_parsed_.begin(), file_contents_parsed_.end(),
                [this,&count](std::vector<double> &pose)
  {
    tuw_object_msgs::ObjectWithCovariance obj;
    obj.object.shape = tuw_object_msgs::Object::SHAPE_DOOR;
    obj.object.pose.position.x = pose[0];
    obj.object.pose.position.y = pose[1];
    obj.object.pose.position.z = pose[2];

    double c_theta = cos(pose[5] * (M_PI / 180.0));
    double s_theta = sin(pose[5] * (M_PI / 180.0));
    Eigen::Matrix3d R;
    R << c_theta, -s_theta, 0,
         s_theta, c_theta, 0,
         0, 0, 1;
    Eigen::Quaterniond q_obj(R);
    obj.object.pose.orientation.x = q_obj.x();
    obj.object.pose.orientation.y = q_obj.y();
    obj.object.pose.orientation.z = q_obj.z();
    obj.object.pose.orientation.w = q_obj.w();
    obj.object.ids = {count};
    obj.object.ids_confidence = {1.0};
    obj.object.shape_variables = {pose[3], pose[4], pose[5], pose[6], pose[7]}; //width, height, opening angle, nr door leafs, clockwise, counterclockwise
    msg_.objects.push_back(obj);
    count++;
  });

  return true;
}
