#include "object_publisher_node.h"
#include "base_pub_object.h"
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include "object_facade.h"

using namespace tuw;
typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

ObjectPublisher::ObjectPublisher(ros::NodeHandle &n)
{
  nh_ = n;
}

void ObjectPublisher::init()
{
  std::string dir;
  std::vector<std::string> obj_types;
  std::vector<std::string> obj_file_locations;
  std::vector<std::string> obj_publisher_topic;
  bool params_correct = nh_.getParam("package_dir", dir);
  params_correct &= nh_.getParam("object_types", obj_types);
  params_correct &= nh_.getParam("object_file_locations", obj_file_locations);
  params_correct &= nh_.getParam("object_publisher_topic", obj_publisher_topic);
  if (!params_correct)
  {
    throw std::runtime_error("Error, supply correct parameters: package_dir [str], \n\r    object_types [list], \n\r    object_file_locations [list], \n\r    object_publisher_topic [list]");
  }
  nr_objects = std::min(std::min(obj_types.size(), obj_file_locations.size()), obj_publisher_topic.size());

  objects_.clear();

  for (int i=0; i < nr_objects; i++)
  {
    objects_.push_back(ObjectFacade::construct(obj_types[i], dir + obj_file_locations[i], obj_publisher_topic[i]));
  }

  for (const std::unique_ptr<BasePubObject> &obj : objects_)
  {
    obj->read();
    obj->createMsg();
    obj->setPublisher(nh_);
  }
}

std::vector<std::unique_ptr<BasePubObject>> &ObjectPublisher::getObjects()
{
  return objects_;
}

void ObjectPublisher::publish()
{
  for (const std::unique_ptr<BasePubObject> &obj : objects_)
  {
    obj->publish();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_publisher_node");
  ros::NodeHandle n;
  ObjectPublisher object_publisher_node(n);
  object_publisher_node.init();
  ros::Rate rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    object_publisher_node.publish();
    for (const std::unique_ptr<BasePubObject> &obj : object_publisher_node.getObjects())
    {
      obj->read();
      obj->createMsg();
    }
    rate.sleep();
  }

  return 0;
}

//void ObjectPublisher::rotate(int i, double rad)
//{
//  Eigen::Quaterniond q_obj;
//  q_obj.x() = door_detection_msg_.objects[i].object.pose.orientation.x;
//  q_obj.y() = door_detection_msg_.objects[i].object.pose.orientation.y;
//  q_obj.z() = door_detection_msg_.objects[i].object.pose.orientation.z;
//  q_obj.w() = door_detection_msg_.objects[i].object.pose.orientation.w;
//  Eigen::Matrix3d Rz = BasePubObject::rotation_matrix_z(rad);
//  Eigen::Matrix3d R = Rz * q_obj.toRotationMatrix();
//  q_obj = Eigen::Quaterniond(R);
//  door_detection_msg_.objects[i].object.pose.orientation.x = q_obj.x();
//  door_detection_msg_.objects[i].object.pose.orientation.y = q_obj.y();
//  door_detection_msg_.objects[i].object.pose.orientation.z = q_obj.z();
//  door_detection_msg_.objects[i].object.pose.orientation.w = q_obj.w();
//}

//void ObjectPublisher::add_position(int i, Eigen::Vector3d position)
//{
//  door_detection_msg_.objects[i].object.pose.position.x += position[0];
//  door_detection_msg_.objects[i].object.pose.position.y += position[1];
//  door_detection_msg_.objects[i].object.pose.position.z += position[2];
//}
