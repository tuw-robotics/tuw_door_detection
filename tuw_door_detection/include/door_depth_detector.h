#ifndef DOOR_DEPTH_DETECTOR_H
#define DOOR_DEPTH_DETECTOR_H

#include "door_detector.h"
#include <ros/publisher.h>

namespace tuw {
class DoorDepthDetector : public DoorDetector
{
public: 
	DoorDepthDetector();
	virtual ~DoorDepthDetector();

protected:
	void processLaser(const sensor_msgs::LaserScan &_laser) override;

private:
	float thresh_{0.2};
	ros::Publisher pubObjectDetections_;
	
};
}

#endif // DOOR_DEPTH_DETECTOR_H
