#ifndef DOOR_DETECTOR_H
#define DOOR_DETECTOR_H

#include <sensor_msgs/LaserScan.h>

namespace tuw {

class DoorDetector {
public:
	DoorDetector() {}
	virtual ~DoorDetector() {}

protected:
	virtual void processLaser(const sensor_msgs::LaserScan &_laser) = 0;
};

}

#endif // DOOR_DETECTOR_H
