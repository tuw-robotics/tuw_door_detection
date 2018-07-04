#include "base_pub_object.h"
#include <boost/tokenizer.hpp>
#include <fstream>

typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;
using namespace tuw;

BasePubObject::BasePubObject(std::string &type, std::string &file_path, std::string &publisher_topic) : type_(type), file_path_(file_path), publisher_topic_name_(publisher_topic), nr_line_parameters(0)
{}

BasePubObject::~BasePubObject()
{}

void BasePubObject::setPublisher(ros::NodeHandle &nh)
{
  pub_ = nh.advertise<tuw_object_msgs::ObjectDetection>(publisher_topic_name_, 5);
}

void BasePubObject::publish()
{
  pub_.publish(msg_);
}

bool BasePubObject::read()
{
  using namespace boost;
  using namespace std;

  ifstream in(file_path_.c_str());
  if (!in.is_open())
  {
    std::cout << "Object read: File path is wrong" << std::endl;
    return false;
  }

  string line;
  file_contents_.clear();

  while(getline(in,line))
  {
    file_contents_.push_back(vector<string>());
    Tokenizer tok(line);

//    if (std::distance(tok.begin(),tok.end()) != nr_line_parameters)
//      throw runtime_error("number of entries in csv file wrong. Must provide position (3 variables) plus shape variables (4) for each door.");

    for_each(tok.begin(),tok.end(), [this](string elem) {
      file_contents_.back().push_back(elem);
    });
  }
  return true;
}

double BasePubObject::deg2rad(int degrees)
{
  return degrees * (M_PI / 180.0);
}

Eigen::Matrix3d BasePubObject::rotation_matrix_z(double rad)
{
  Eigen::Matrix3d R;
  R << cos(rad), -sin(rad), 0,
       sin(rad), cos(rad), 0,
       0,   0,    1;
  return R;
}
