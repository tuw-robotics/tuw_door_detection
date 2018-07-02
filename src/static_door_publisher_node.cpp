#include "static_door_publisher_node.h"
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>

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
  int count;
  std::for_each(door_positions_.begin(), door_positions_.end(),
                [this,&count](std::vector<double> &pose)
  {
    tuw_object_msgs::ObjectWithCovariance obj;
    obj.object.shape = tuw_object_msgs::Object::SHAPE_DOOR;
    obj.object.pose.position.x = pose[0];
    obj.object.pose.position.y = pose[1];
    obj.object.pose.position.z = pose[2];
    obj.object.pose.orientation.x = 0;
    obj.object.pose.orientation.y = 0;
    obj.object.pose.orientation.z = 0;
    obj.object.pose.orientation.w = 1.0;
    obj.object.ids = {count};
    obj.object.ids_confidence = {1.0};
    obj.object.shape_variables = {pose[3], pose[4], pose[5], pose[6]}; //width, height, opening angle, nr door leafs
    door_detection_msg_.objects.push_back(obj);
    count++;
  });
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
  ros::Rate rate(5);

  while (ros::ok())
  {
    ros::spinOnce();
    static_door_publisher_node.publishDoors();
    rate.sleep();
  }

  return 0;
}
