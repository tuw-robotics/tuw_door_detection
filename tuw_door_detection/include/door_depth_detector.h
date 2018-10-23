#ifndef DOOR_DEPTH_DETECTOR_H
#define DOOR_DEPTH_DETECTOR_H

#include "door_detector.h"
#include <ros/publisher.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace tuw {
class DoorDepthDetector : public DoorDetector
{

public: 
	struct ParametersNode {
		ParametersNode();
		ros::NodeHandle node;
		bool debug;
		std::string world_frame;
		std::string source_frame;
		std::string publisher_topic;
		std::string internal_mode;
	};
	
	std::unique_ptr<ParametersNode> params_;

	DoorDepthDetector(ros::NodeHandle &_nh);
	virtual ~DoorDepthDetector();

protected:
	bool processLaser(const sensor_msgs::LaserScan &_laser) override;

private:
	
	float thresh_{0.3};
	std::size_t KERNEL_SIZE={16};
	ros::Publisher pubObjectDetections_;

	std::unique_ptr<ParametersNode> &params() { return params_; }
	bool structureMode(const sensor_msgs::LaserScan &_laser, std::vector<Eigen::Vector2d> &_detections);
	bool kernelMode(const sensor_msgs::LaserScan &_laser, std::vector<Eigen::Vector2d> &_detections);
};
}

#endif // DOOR_DEPTH_DETECTOR_H
