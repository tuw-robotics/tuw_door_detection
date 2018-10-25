#ifndef DOOR_DETECTION_HPP
#define DOOR_DETECTION_HPP

#include <tuw_geometry/measurement_laser.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>

namespace tuw {

class DoorDetection;
using DoorDetectionPtr = std::shared_ptr<DoorDetection>;
using DoorDetectionConstPtr = std::shared_ptr<DoorDetection const>;

class DoorDetection : public MeasurementLaser
{
	public:
		DoorDetection();
		
		double &response() { return response_; }
		double &responseNormalized() { return response_normalized_; }
		bool &validDetection() { return valid_; }
		
		const	double &response() const { return response_; }
		const	double &responseNormalized() const { return response_normalized_; }
		const	bool &validDetection() const { return valid_; }
		
		tuw_object_msgs::ObjectWithCovariance toMsg(int id);
		
		friend std::ostream &operator<<(std::ostream &output, const DoorDetection &d);
		
	protected:
		double response_;	
		double response_normalized_;
		bool valid_;
};
    
};

#endif
