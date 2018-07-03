#include "object_publisher_node.h"
#include "base_pub_object.h"
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Geometry>

using namespace tuw;
typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

ObjectPublisher::ObjectPublisher(ros::NodeHandle &n)
{
  nh_ = n;
}

template<typename T>
void print_v(std::vector<T> &v)
{
  for (auto e : v)
    std::cout << ", " << e;
  std::cout << std::endl;
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
}

void ObjectPublisher::rotate(int i, double rad)
{
  Eigen::Quaterniond q_obj;
  q_obj.x() = door_detection_msg_.objects[i].object.pose.orientation.x;
  q_obj.y() = door_detection_msg_.objects[i].object.pose.orientation.y;
  q_obj.z() = door_detection_msg_.objects[i].object.pose.orientation.z;
  q_obj.w() = door_detection_msg_.objects[i].object.pose.orientation.w;
  Eigen::Matrix3d Rz = BasePubObject::rotation_matrix_z(rad);
  Eigen::Matrix3d R = Rz * q_obj.toRotationMatrix();
  q_obj = Eigen::Quaterniond(R);
  door_detection_msg_.objects[i].object.pose.orientation.x = q_obj.x();
  door_detection_msg_.objects[i].object.pose.orientation.y = q_obj.y();
  door_detection_msg_.objects[i].object.pose.orientation.z = q_obj.z();
  door_detection_msg_.objects[i].object.pose.orientation.w = q_obj.w();
}

void ObjectPublisher::add_position(int i, Eigen::Vector3d position)
{
  door_detection_msg_.objects[i].object.pose.position.x += position[0];
  door_detection_msg_.objects[i].object.pose.position.y += position[1];
  door_detection_msg_.objects[i].object.pose.position.z += position[2];
}

void ObjectPublisher::publishDoors()
{
  door_pub_.publish(door_detection_msg_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_door_publisher_node");
  ros::NodeHandle n;
  ObjectPublisher static_door_publisher_node(n);
  static_door_publisher_node.init();
  ros::Rate rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    static_door_publisher_node.init();
    static_door_publisher_node.publishDoors();
    //static_door_publisher_node.add_position(0, Eigen::Vector3d(0.1,0.1,0.0));
    //static_door_publisher_node.rotate(0,2.0 * (M_PI / 180.0));
    rate.sleep();
  }

  return 0;
}
