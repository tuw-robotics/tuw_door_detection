#include "base_pub_object.h"
#include <boost/tokenizer.hpp>
#include <fstream>

typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

tuw::BasePubObject::BasePubObject(std::string &type, std::string &file_path, std::string &publisher_topic) : type_(type), file_path_(file_path), publisher_topic_name_(publisher_topic)
{}

tuw::BasePubObject::~BasePubObject()
{}

void tuw::BasePubObject::setPublisher(ros::NodeHandle &nh)
{
  pub_ = nh.advertise<tuw_object_msgs::ObjectDetection>(publisher_topic_name_, 5);
}

void tuw::BasePubObject::publish()
{
  pub_.publish(msg_);
}

bool tuw::BasePubObject::read()
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
    if (line[0] == '#')
    {
      continue; // allows simple comments
    }
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

void tuw::BasePubObject::tf2pose(tf::Transform &_tf, geometry_msgs::Pose &p)
{
  p.position.x = _tf.getOrigin().getX();
  p.position.y = _tf.getOrigin().getY();
  p.position.z = _tf.getOrigin().getZ();

  p.orientation.x = _tf.getRotation().getX();
  p.orientation.y = _tf.getRotation().getY();
  p.orientation.z = _tf.getRotation().getZ();
  p.orientation.w = _tf.getRotation().getW();
}

void tuw::BasePubObject::pose2tf(geometry_msgs::Pose &p, tf::Transform &_tf)
{
  tf::Vector3 t(p.position.x, p.position.y, p.position.z);
  tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

  _tf.setOrigin(t);
  _tf.setRotation(q);
}

bool tuw::BasePubObject::restore_tf()
{
  if (old_tfs_.size() != msg_.objects.size())
  {
    return false;
  }

  int obj_idx=0;

  for (auto obj_it : msg_.objects)
  {
    obj_it.object.pose = old_tfs_[obj_idx];
    ++obj_idx;
  }

  return true;
}

void tuw::BasePubObject::set_pose_from_tf(tf::StampedTransform &_tf)
{
  for (auto &obj_it : msg_.objects)
  {
    msg_.header.stamp = _tf.stamp_;
    msg_.header.frame_id = _tf.frame_id_;

    geometry_msgs::Pose &p = obj_it.object.pose;

    tf2pose(_tf, p);

  }
}

bool tuw::BasePubObject::multiply_tf(tf::StampedTransform &_tf, bool _keep_old)
{
  old_tfs_.resize(msg_.objects.size());

  int obj_idx = 0;
  msg_.header.frame_id = _tf.frame_id_;
  msg_.header.stamp = _tf.stamp_;

  for (auto &obj_it : msg_.objects)
  {
    geometry_msgs::Pose &p = obj_it.object.pose;
    tf::Transform tf_object;

    pose2tf(p, tf_object);

    if (_keep_old)
    {
      old_tfs_[obj_idx] = obj_it.object.pose;
    }

    //tf_object.getOrigin() += _tf.getOrigin();
    tf_object.mult(_tf, tf_object);

    tf2pose(tf_object, p);

    ++obj_idx;
  }
  return true;
}

const std::string tuw::BasePubObject::as_string()
{
  std::stringstream strm("");
  for (auto &o : msg_.objects)
  {
    strm << "id: " << o.object.ids[0] << "\n";
    strm << "pose " << "(" << std::to_string(o.object.pose.position.x) << ", " << std::to_string(o.object.pose.position.y) << ", " << std::to_string(o.object.pose.position.z) << "), (" << std::to_string(o.object.pose.orientation.x) << ", " << std::to_string(o.object.pose.orientation.y) << ", " << std::to_string(o.object.pose.orientation.z) << ", " << std::to_string(o.object.pose.orientation.w) << ")" << "\n";
  }
  return strm.str();
}
