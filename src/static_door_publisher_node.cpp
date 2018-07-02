#include "static_door_publisher_node.h"
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Geometry>

using namespace tuw;
typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

StaticDoorPublisherNode::StaticDoorPublisherNode(ros::NodeHandle &n)
{
  nh_ = n;
  door_pub_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("doors_detected", 10);
}

void StaticDoorPublisherNode::init()
{
  StaticDoorPublisherNode::readFile();
  StaticDoorPublisherNode::prepareMsgs();
}

void StaticDoorPublisherNode::prepareMsgs()
{
  door_detection_msg_ = tuw_object_msgs::ObjectDetection();
  door_detection_msg_.header.stamp = ros::Time::now();
  door_detection_msg_.header.frame_id = std::string("map");
  door_detection_msg_.type = std::string("");
  int count = 0;
  std::for_each(door_positions_.begin(), door_positions_.end(),
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
    door_detection_msg_.objects.push_back(obj);
    count++;
  });
}

double StaticDoorPublisherNode::deg2rad(int degrees)
{
  return degrees * (M_PI / 180.0);
}

void StaticDoorPublisherNode::rotate(int i, double rad)
{
  Eigen::Quaterniond q_obj;
  q_obj.x() = door_detection_msg_.objects[i].object.pose.orientation.x;
  q_obj.y() = door_detection_msg_.objects[i].object.pose.orientation.y;
  q_obj.z() = door_detection_msg_.objects[i].object.pose.orientation.z;
  q_obj.w() = door_detection_msg_.objects[i].object.pose.orientation.w;
  Eigen::Matrix3d Rz = rotation_matrix_z(rad);
  Eigen::Matrix3d R = Rz * q_obj.toRotationMatrix();
  q_obj = Eigen::Quaterniond(R);
  door_detection_msg_.objects[i].object.pose.orientation.x = q_obj.x();
  door_detection_msg_.objects[i].object.pose.orientation.y = q_obj.y();
  door_detection_msg_.objects[i].object.pose.orientation.z = q_obj.z();
  door_detection_msg_.objects[i].object.pose.orientation.w = q_obj.w();
}

void StaticDoorPublisherNode::add_position(int i, Eigen::Vector3d position)
{
  door_detection_msg_.objects[i].object.pose.position.x += position[0];
  door_detection_msg_.objects[i].object.pose.position.y += position[1];
  door_detection_msg_.objects[i].object.pose.position.z += position[2];
}

Eigen::Matrix3d StaticDoorPublisherNode::rotation_matrix_z(double rad)
{
  Eigen::Matrix3d R;
  R << cos(rad), -sin(rad), 0,
       sin(rad), cos(rad), 0,
       0,   0,    1;
  return R;
}

void StaticDoorPublisherNode::readFile()
{
  door_locations_file_ = "/home/felix/projects/catkin/tuw/src/tuw_door_detection/files/doors.csv";
  using namespace boost;
  using namespace std;

  ifstream in(door_locations_file_.c_str());
  if (!in.is_open())
  {
    throw runtime_error("StaticDoorPublisherNode: File path is wrong");
  }

  string line;
  door_positions_.clear();

  while(getline(in,line))
  {
    door_positions_.push_back(std::vector<double>());
    Tokenizer tok(line);

    if (std::distance(tok.begin(),tok.end()) != 7)
      throw runtime_error("number of entries in csv file wrong. Must provide position (3 variables) plus shape variables (4) for each door.");

    for_each(tok.begin(),tok.end(), [this](std::string elem) {
      door_positions_.back().push_back(stod(elem));
    });
  }
}

void StaticDoorPublisherNode::publishDoors()
{
  door_pub_.publish(door_detection_msg_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_door_publisher_node");
  ros::NodeHandle n;
  StaticDoorPublisherNode static_door_publisher_node(n);
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
