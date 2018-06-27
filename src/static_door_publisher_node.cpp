#include "static_door_publisher_node.h"
#include <boost/tokenizer.hpp>
#include <fstream>

using namespace tuw;
typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

StaticDoorPublisherNode::StaticDoorPublisherNode(ros::NodeHandle &n)
{
  nh_ = n;
}

void StaticDoorPublisherNode::init()
{
  StaticDoorPublisherNode::readFile();

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

  while(getline(in,line))
  {
    door_positions_.push_back(std::vector<double>());
    Tokenizer tok(line);
    for_each(tok.begin(),tok.end(), [this](std::string elem) {
      door_positions_.back().push_back(stod(elem));
    });
  }
}

void StaticDoorPublisherNode::publishDoors()
{
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "linesegment2d_detector_node");
  ros::NodeHandle n;
  StaticDoorPublisherNode static_door_publisher_node(n);
  static_door_publisher_node.init();
  ros::Rate rate(5);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
