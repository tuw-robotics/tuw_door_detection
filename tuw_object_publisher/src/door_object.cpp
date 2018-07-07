#include "door_object.h"
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

using namespace tuw;

DoorObject::DoorObject(std::string &type,std::string &file_path, std::string &publisher_topic) : BasePubObject(type, file_path, publisher_topic)
{}

DoorObject::~DoorObject(){}

bool DoorObject::createMsg()
{
  file_contents_parsed_.clear();
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
    obj.object.pose.position.x = pose[static_cast<int>(DoorObjectConstants::Position_x)];
    obj.object.pose.position.y = pose[static_cast<int>(DoorObjectConstants::Position_y)];
    obj.object.pose.position.z = pose[static_cast<int>(DoorObjectConstants::Position_z)];
    double w_angle = pose[static_cast<int>(DoorObjectConstants::angle_w)] * (M_PI / 180.0);
    double d_angle = pose[static_cast<int>(DoorObjectConstants::angle_d)] * (M_PI / 180.0);
    double c_theta = cos(w_angle);
    double s_theta = sin(w_angle);
    Eigen::Matrix3d R_wd; //rotation from door reference frame to world frame
    R_wd << c_theta, -s_theta, 0,
         s_theta, c_theta, 0,
         0, 0, 1;
    Eigen::Quaterniond q_obj(R_wd);
    obj.object.pose.orientation.x = q_obj.x();
    obj.object.pose.orientation.y = q_obj.y();
    obj.object.pose.orientation.z = q_obj.z();
    obj.object.pose.orientation.w = q_obj.w();
    obj.object.ids = {count};
    obj.object.ids_confidence = {1.0};
    obj.object.shape_variables = {pose[static_cast<int>(DoorObjectConstants::width)],
                                  pose[static_cast<int>(DoorObjectConstants::height)],
                                  w_angle,
                                  d_angle,
                                  pose[static_cast<int>(DoorObjectConstants::leaves)],
                                  pose[static_cast<int>(DoorObjectConstants::clockwise)]}; //width, height, angle (in world reference frame), opening angle (in door reference frame!), nr door leafs, clockwise
    msg_.objects.push_back(obj);
    count++;
  });

  return true;
}
