#include "object_publisher_node.h"
#include "base_pub_object.h"
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include "object_facade.h"

typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

tuw::ObjectPublisher::ObjectPublisher(ros::NodeHandle &n)
{
  nh_ = n;
}

void tuw::ObjectPublisher::init()
{
  std::string dir;
  std::vector<std::string> obj_types;
  std::vector<std::string> obj_file_locations;
  std::vector<std::string> obj_publisher_topic;
  bool params_correct = nh_.getParam("package_dir", dir);
  params_correct &= nh_.getParam("object_types", obj_types);
  params_correct &= nh_.getParam("object_file_locations", obj_file_locations);
  params_correct &= nh_.getParam("object_publisher_topic", obj_publisher_topic);
  params_correct &= nh_.getParam("robot_perspective", robot_perspective_);
  params_correct &= nh_.getParam("debug", debug_);

  if (!params_correct)
  {
    throw std::runtime_error("Error, supply correct parameters: package_dir [str], \n\r    object_types [list], \n\r    object_file_locations [list], \n\r    object_publisher_topic [list]");
  }
  nr_objects_ = std::min(std::min(obj_types.size(), obj_file_locations.size()), obj_publisher_topic.size());

  objects_.clear();

  for (unsigned int i=0; i < nr_objects_; i++)
  {
    objects_.push_back(ObjectFacade::construct(obj_types[i], dir + obj_file_locations[i], obj_publisher_topic[i]));
  }

  if (robot_perspective_)
  {
    tf::StampedTransform tf;
    dummy_ = tuw::ObjectFacade::create(tuw_object_msgs::ObjectDetection::OBJECT_TYPE_DOOR, tf);
    dummy_->setPublisher(nh_);
  }

  for (const std::unique_ptr<BasePubObject> &obj : objects_)
  {
    obj->read();
    obj->createMsg();
    obj->setPublisher(nh_);
  }
  ROS_INFO("init done, objects are being published");
}

std::vector<std::unique_ptr<tuw::BasePubObject>> &tuw::ObjectPublisher::getObjects()
{
  return objects_;
}

void tuw::ObjectPublisher::publish()
{
  tf::StampedTransform tf_des;
  std::string target_frame = tf::resolve("", "r0/laser0");

  if (robot_perspective_)
  {
    std::string source_frame = tf::resolve("", "/map");
    //T_LM -> transformation from map to laser (from source to target)
    waitForTransform(tf_des, target_frame, source_frame, ros::Time(0), ros::Duration(10,0));
    if (debug_)
    {
      ROS_INFO("tf t: (%lf,%lf,%lf)", tf_des.getOrigin().getX(), tf_des.getOrigin().getY(), tf_des.getOrigin().getZ());
      ROS_INFO("tf q: (%lf,%lf,%lf,%lf)", tf_des.getRotation().getX(), tf_des.getRotation().getY(), tf_des.getRotation().getZ(), tf_des.getRotation().getW());
    }
  }

  for (const std::unique_ptr<BasePubObject> &obj : objects_)
  {
    if (robot_perspective_)
    {
      obj->multiply_tf(tf_des, true);
      obj->getMsg().header.frame_id = target_frame;
      obj->getMsg().header.stamp = tf_des.stamp_;
    }

    obj->publish();

    if (robot_perspective_)
    {
      obj->restore_tf();
    }
  }
}

bool tuw::ObjectPublisher::waitForTransform(
    tf::StampedTransform &des, const std::string& target_frame,
    const std::string& source_frame, const ros::Time& time, const ros::Duration& polling_sleep_duration)
{
    try
    {
        if (debug_)
            ROS_INFO(
                "debug: waitForTransform(): target_frame='%s' "
                "source_frame='%s'",
                target_frame.c_str(), source_frame.c_str());

        listenerTF_.waitForTransform(
            target_frame, source_frame, time, polling_sleep_duration);
        listenerTF_.lookupTransform(
            target_frame, source_frame, time, des);
    }
    catch (tf::TransformException)
    {
        ROS_INFO(
            "Failed to get transform target_frame (%s) to source_frame (%s)",
            target_frame.c_str(), source_frame.c_str());
        return false;
    }

    return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_publisher_node");
  ros::NodeHandle n;
  tuw::ObjectPublisher object_publisher_node(n);
  object_publisher_node.init();
  ros::Rate rate(60);

  while (ros::ok())
  {
    ros::spinOnce();
    object_publisher_node.publish();
//    for (const std::unique_ptr<tuw::BasePubObject> &obj : object_publisher_node.getObjects())
//    {
//      obj->read();
//      obj->createMsg();
//    }
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
