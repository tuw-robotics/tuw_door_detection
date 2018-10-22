#include "door_detector.h"
#include <tuw_object_msgs/ObjectDetection.h>

using namespace tuw;

DoorDetector::DoorDetector(ros::NodeHandle &_nh) : nh_(_nh)
{
}

DoorDetector::~DoorDetector()
{}

void DoorDetector::init(const std::string &pub_topic)
{
	pubObjectDetections_ = nh_.advertise<tuw_object_msgs::ObjectDetection>(pub_topic, 1, true);
}

void DoorDetector::publish()
{
	if (!objects_.size())
	{
		return;
	}
	
	tuw_object_msgs::ObjectDetection obj_detections;
	//obj_detections.header = _scan.header;
	obj_detections.header = last_header_;
	obj_detections.objects.resize(objects_.size());

	std::size_t obj_count = 0;
	for (const auto &obj : objects_)
	{
		obj_detections.objects[obj_count] = std::move(obj);
		obj_count++;
	}
	
	pubObjectDetections_.publish(obj_detections);
	
	objects_.clear();
}

Eigen::Matrix<double,4,4> DoorDetector::tf2EigenMat(const tf::Transform &tf)
{
	Eigen::Matrix<double,4,4> m;
	const auto r = tf.getBasis();
	const auto t = tf.getOrigin();
	m << r[0][0], r[0][1], r[0][2], t[0],
	     r[1][0], r[1][1], r[1][2], t[1],
	     r[2][0], r[1][1], r[2][2], t[2],
			 0, 0, 1;
	return m;
}

bool DoorDetector::getTF(const std::string &world_frame, const std::string &source_frame, tf::StampedTransform &_pose, bool debug) 
{
  std::string target_frame_id = source_frame;
  std::string source_frame_id = tf::resolve("", world_frame);
  std::string key = target_frame_id + "->" + source_frame_id;
  
  if (debug)
  {
		ROS_INFO("lookUpTransform %s", key.c_str());
  }

  try
  {
    listenerTF_.lookupTransform(
        target_frame_id, source_frame_id, ros::Time(0), _pose);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("getStaticTF");
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  
  return true;
}
