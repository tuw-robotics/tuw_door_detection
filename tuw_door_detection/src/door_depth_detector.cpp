#include "door_depth_detector.h"
#include <eigen3/Eigen/Core>

using namespace tuw;

DoorDepthDetector::DoorDepthDetector() : DoorDetector()
{
}

DoorDepthDetector::~DoorDepthDetector()
{
}


void DoorDepthDetector::processLaser(const sensor_msgs::LaserScan &_scan)
{
	size_t N = _scan.ranges.size();
	float last_range = _scan.ranges[0];	
	std::vector<Eigen::Vector2f> outliers; 

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
        outliers.push_back(Eigen::Vector2f());
        outliers.back()[0] = cos ( angle ) * length;
        outliers.back()[1] = sin ( angle ) * length;
		}

		last_range = length;
	}
}
