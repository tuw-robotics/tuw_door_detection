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
	node.param<std::string>("internal_mode", internal_mode, "kernel");
}

DoorDepthDetector::DoorDepthDetector(ros::NodeHandle &_nh) : params_(new DoorDepthDetector::ParametersNode()), DoorDetector(_nh)
{
	init(params_->publisher_topic);
}

DoorDepthDetector::~DoorDepthDetector()
{
}

bool DoorDepthDetector::kernelMode(const sensor_msgs::LaserScan &_scan, std::vector<Eigen::Vector2d> &_detections)
{
	
}

bool DoorDepthDetector::structureMode(const sensor_msgs::LaserScan &_scan, std::vector<Eigen::Vector2d> &_detections)
{
	//Eigen::Matrix<double,4,4> tf_laser2world = tf2EigenMat(tf_laser); 
	//std::cout << "robot pose: (" << tf_laser2world(0,3) << ", " << tf_laser2world(1,3) << ")" << std::endl;
	
	size_t N = _scan.ranges.size();
	std::cout << "ranges size: " << N << std::endl;
	
	int i=0;
	float last_range = _scan.ranges[0];
	int last_range_idx = 0;
	while(!isfinite(last_range) && i < N)
	{
		i++;
		last_range = _scan.ranges[i];	
		last_range_idx = i;
	}
	
	std::vector<Eigen::Vector2d> detected_door_ranges;
	std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> candidates;
	std::vector<std::pair<unsigned int, unsigned int>> candidates_idx;

	for (; i < N; ++i)
	{
		double length = _scan.ranges[i];
		double abs_diff = fabs(length - last_range);

    if ( ( length < _scan.range_max ) && isfinite ( length ) && abs_diff > thresh_ ) 
		{
				std::cout << "range diff: " << abs_diff << std::endl;
				std::cout << "range: " << length << std::endl;
				std::cout << "last range: " << last_range << std::endl;
				
				Eigen::Vector2d meas1 = range2Eigen(_scan, i);
				Eigen::Vector2d meas0 = range2Eigen(_scan, last_range_idx);
				
				candidates_idx.push_back(std::make_pair(last_range_idx, i));
				candidates.push_back(std::make_pair(meas0, meas1));
		}

		if (isfinite(length))
		{
			last_range = length;
			last_range_idx = i;
		}
	}
	
	std::size_t idx_start = candidates_idx[0].second;
	std::size_t idx_candidates = 1;
	for (; idx_candidates < candidates_idx.size(); ++idx_candidates)
	{
	}
	
	//
	std::set<std::size_t> idx_set;
	i = 0;
	for (const auto &c : candidates_idx)
	{
			if (idx_set.count(c.first) == 0)
			{
				_detections.push_back(candidates[i].first);
				idx_set.insert(c.first);
			}
			
			if (idx_set.count(c.second) == 0)
			{
				_detections.push_back(candidates[i].second);
				idx_set.insert(c.second);
			}
			i++;
	}
}

bool DoorDepthDetector::processLaser(const sensor_msgs::LaserScan &_scan)
{
	
	last_header_.reset(new std_msgs::Header(_scan.header));
	objects_.clear();
	
	tf::StampedTransform tf_laser;
	if (!getTF(params()->world_frame, params()->source_frame, tf_laser, params()->debug))
	{
		return false;
	}
	
	std::vector<Eigen::Vector2d> detections;
	if (params()->internal_mode == "structure")
	{
		structureMode(_scan, detections);
	}
	else 
	{
		kernelMode(_scan, detections);
	}
	
	//t = tf_laser2world * t;
	for (Eigen::Vector2d &v : detections)
	{	
			Eigen::Quaterniond q(1,0,0,0);
			tuw_object_msgs::ObjectWithCovariance out_obj;
			out_obj.object.pose.position.x = v[0];				
			out_obj.object.pose.position.y = v[1];				
			out_obj.object.pose.position.z = 0;//v[2];				
			
			out_obj.object.pose.orientation.x = q.x();
			out_obj.object.pose.orientation.y = q.y();
			out_obj.object.pose.orientation.z = q.z();
			out_obj.object.pose.orientation.w = q.w();
			
			out_obj.object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
			out_obj.object.ids = { static_cast<int>(objects_.size()) };
			out_obj.object.ids_confidence = {0};
			out_obj.object.shape_variables = {0.1, 1, 0.5}; 
			objects_.push_back(std::move(out_obj));
	}
	
	return true;
}


