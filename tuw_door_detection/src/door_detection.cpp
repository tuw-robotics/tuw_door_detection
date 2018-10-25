#include "door_detection.h"

using namespace tuw;

DoorDetection::DoorDetection()
{
	this->response_ = std::nan("uninitialized");
	this->response_normalized_ = std::nan("uninitialized");
	this->valid_ = false;
}

tuw_object_msgs::ObjectWithCovariance DoorDetection::toMsg(int id)
{
	tuw_object_msgs::ObjectWithCovariance dd;
	
	//TODO refine accordingly (more beams per door measurement i.e. all beams connected to the door)
	dd.object.pose.position.x = this->operator[](0).end_point.x();
	dd.object.pose.position.y = this->operator[](0).end_point.y();
	dd.object.pose.position.z = 0;
	
	dd.object.pose.orientation.x = 0;
	dd.object.pose.orientation.y = 0;
	dd.object.pose.orientation.z = 0;
	dd.object.pose.orientation.w = 1;
	
	//TODO: dummy for now
	dd.object.shape = tuw_object_msgs::Object::SHAPE_CONE;
	dd.object.ids = { static_cast<int>(id) };
	dd.object.ids_confidence = {0};
	dd.object.shape_variables = {0.1, 1, 0.5}; 
	
	return std::move(dd);
}

namespace tuw {
	std::ostream &operator<<( std::ostream &output, const DoorDetection &d ) 
	{ 
		output << "Beams: {(" << d[0].end_point.x() << ", " << d[0].end_point.y() << ")";
		for (int i=1; i < d.size(); ++i)
		{
			output << ", (" << d[i].end_point.x() << ", " << d[i].end_point.y() << ")";
		}
		output << "}" << "\n";
		output << "resp: " << std::to_string(d.response()) << "\n";
		output << "resp_n: " << std::to_string(d.responseNormalized()) << "\n";
		output << "valid_det: " << (d.validDetection() ? "true" : "false") << "\n";
	
		return output;            
	};		     
}
