#include "cone_object.h"
#include <eigen3/Eigen/Geometry>

using namespace tuw;

ConeObject::ConeObject(std::string &type, std::string &file_path, std::string &publisher_topic) : BasePubObject(type, file_path, publisher_topic)
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
    obj.object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
    obj.object.pose.position.x = pose[0];
    obj.object.pose.position.y = pose[1];
    obj.object.pose.position.z = pose[2];
    obj.object.pose.orientation.x = 0;
    obj.object.pose.orientation.y = 0;
    obj.object.pose.orientation.z = 0;
    obj.object.pose.orientation.w = 1.0;
    obj.object.ids = {count};
    obj.object.ids_confidence = {1.0};
    obj.object.shape_variables = {pose[3], pose[4], pose[5]}; //radius, color, height
    msg_.objects.push_back(obj);
    count++;
  });

  return true;
}
