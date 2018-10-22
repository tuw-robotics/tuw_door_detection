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
	};
	
	std::unique_ptr<ParametersNode> params_;

	DoorDepthDetector(ros::NodeHandle &_nh);
	virtual ~DoorDepthDetector();

protected:
	bool processLaser(const sensor_msgs::LaserScan &_laser) override;

private:
	
	float thresh_{0.2};
	ros::Publisher pubObjectDetections_;

	std::unique_ptr<ParametersNode> &params() { return params_; }
};
}

#endif // DOOR_DEPTH_DETECTOR_H
