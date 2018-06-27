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
  door_detection_msg_.objects.reserve(door_positions_.size());
  std::cout << "having " << door_positions_.size() << " objs" << std::endl;
  for (auto vec_double : door_positions_)
    std::cout << vec_double[0] << ", " << vec_double[1] << ", " << vec_double[2] << std::endl;
  unsigned int count;
  std::for_each(door_positions_.begin(), door_positions_.end(),
                [this,&count](std::vector<double> &pose)
  {
    tuw_object_msgs::ObjectWithCovariance &obj = door_detection_msg_.objects[count];
    obj.object.shape = tuw_object_msgs::Object::SHAPE_DOOR;
    std::cout << "pose " << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;
    obj.object.pose.position.x = pose[0];
    obj.object.pose.position.y = pose[1];
    obj.object.pose.position.z = pose[2];
    std::cout << "object x " << obj.object.pose.position.x << std::endl;
    std::cout << "object y " << obj.object.pose.position.y << std::endl;
    std::cout << "object z " << obj.object.pose.position.z << std::endl;
    //std::cout << "before obj ids" << std::endl;
    //obj.object.ids = {count};
    //std::cout << "after obj ids" << std::endl;
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
    for_each(tok.begin(),tok.end(), [this](std::string elem) {
      door_positions_.back().push_back(stod(elem));
      std::cout << "elem: " << elem << std::endl;
    });
  }
}

void StaticDoorPublisherNode::publishDoors()
{
  door_pub_.publish(door_detection_msg_);
}

void StaticDoorPublisherNode::callbackObjectDetection(tuw_object_msgs::ObjectDetection &obj_detection)
{
  std::cout << "objects: " << obj_detection.objects.size() << std::endl;
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
