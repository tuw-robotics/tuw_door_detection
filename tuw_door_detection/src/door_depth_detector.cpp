#include "door_depth_detector.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>

using namespace tuw;

DoorDepthDetector::ParametersNode::ParametersNode() : node("~")
{
	node.param<std::string>("source_frame", source_frame, "/base_link");
	node.param<std::string>("world_frame", world_frame, "/map");
	node.param<bool>("debug", debug, false);
	node.param<std::string>("publisher_topic", publisher_topic, "/door_detections");
}

DoorDepthDetector::DoorDepthDetector(ros::NodeHandle &_nh) : params_(new DoorDepthDetector::ParametersNode()), DoorDetector(_nh)
{
	init(params_->publisher_topic);
}

DoorDepthDetector::~DoorDepthDetector()
{
}

bool DoorDepthDetector::processLaser(const sensor_msgs::LaserScan &_scan)
{
	
	last_header_ = _scan.header;
	
	objects_.clear();
	
	tf::StampedTransform tf_laser;
	if (!getTF(params()->world_frame, params()->source_frame, tf_laser, params()->debug))
	{
		return false;
	}
	Eigen::Matrix<double,4,4> tf_laser2world = tf2EigenMat(tf_laser); 
	
	size_t N = _scan.ranges.size();
	float last_range = _scan.ranges[0];	

	for (int i=0; i < N; ++i)
	{
		double length = _scan.ranges[i];
		
		if ( (length - last_range) < thresh_ )
		{
				continue;	
		}

    if ( ( length < _scan.range_max ) && isfinite ( length ) ) 
		{
        double angle  = _scan.angle_min + ( _scan.angle_increment * i );
        Eigen::Vector4d t;
        t[0] = cos ( angle ) * length;
        t[1] = sin ( angle ) * length;
				t[2] = 0.0;
				t[3] = 1.0;
				t = tf_laser2world * t;
				Eigen::Quaterniond q(1,0,0,0);
				tuw_object_msgs::ObjectWithCovariance out_obj;
				out_obj.object.pose.position.x = t[0];				
				out_obj.object.pose.position.y = t[1];				
				out_obj.object.pose.position.z = t[2];				
				
				out_obj.object.pose.orientation.x = 0;
				out_obj.object.pose.orientation.y = 0;
				out_obj.object.pose.orientation.z = 0;
				out_obj.object.pose.orientation.w = 1;
				
				out_obj.object.shape = tuw_object_msgs::Object::SHAPE_CONE;
				out_obj.object.ids = {static_cast<int>(objects_.size())};
				out_obj.object.ids_confidence = {0};
				objects_.push_back(std::move(out_obj));
		}

		last_range = length;
	}

	return true;
}


