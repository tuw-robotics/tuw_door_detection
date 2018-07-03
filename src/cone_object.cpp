#include "cone_object.h"
#include <eigen3/Eigen/Geometry>

using namespace tuw;

ConeObject::ConeObject(std::string &type, std::string &file_path) : BasePubObject(type, file_path)
{}

ConeObject::~ConeObject() {}

bool ConeObject::createMsg()
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
    Eigen::Quaterniond q_obj = Eigen::Quaterniond(rotation_matrix_z(deg2rad(pose[5])));
    obj.object.pose.orientation.x = q_obj.x();
    obj.object.pose.orientation.y = q_obj.y();
    obj.object.pose.orientation.z = q_obj.z();
    obj.object.pose.orientation.w = q_obj.w();
    obj.object.ids = {count};
    obj.object.ids_confidence = {1.0};
    obj.object.shape_variables = {pose[3], pose[4], pose[5], pose[6]}; //width, height, opening angle, nr door leafs
    msg_.objects.push_back(obj);
    count++;
  });

  return true;
}
