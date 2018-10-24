#ifndef DOOR_DEPTH_DETECTOR_H
#define DOOR_DEPTH_DETECTOR_H

#include "door_detector.h"
#include <ros/publisher.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tuw_geometry/figure.h>

namespace tuw {
class DoorDepthDetector : public DoorDetector
{

public: 
	//TODO: USE dynamic reconfigure!
	struct Config {
		double map_pix_x = 500;
		double map_pix_y = 500;
		double map_max_x = 5;
		double map_max_y = 5;
		double map_min_x = -5;
		double map_min_y = -5;
		double map_grid_x = 1;
		double map_grid_y = 1;
		double map_rotation = 0;
	};
	
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
	void plot(const sensor_msgs::LaserScan &_scan, std::vector<float> &_responses);
	Config config_;

private:
	Figure figure_local_;
	float thresh_{0.3};
	std::size_t KERNEL_SIZE = {8};
	ros::Publisher pubObjectDetections_;

	std::unique_ptr<ParametersNode> &params() { return params_; }
	bool structureMode(const sensor_msgs::LaserScan &_laser, std::vector<Eigen::Vector2d> &_detections);
	bool kernelMode(const sensor_msgs::LaserScan &_laser, std::vector<Eigen::Vector2d> &_detections);
};
}

#endif // DOOR_DEPTH_DETECTOR_H
