//
// Created by felix on 11.04.19.
//

#ifndef TUW_DOOR_DETECTION_OBJECT_SENSOR_MODEL_EVALUATOR_H
#define TUW_DOOR_DETECTION_OBJECT_SENSOR_MODEL_EVALUATOR_H

#include <tuw_object_msgs/ObjectDetection.h>
#include <datastructures/octo_object_map.h>
#include <Eigen/Core>

namespace tuw
{
  class ObjectSensorModel
  {
  public:
    ObjectSensorModel();
    
    ~ObjectSensorModel() = default;
    
    void process( tuw_object_msgs::ObjectWithCovariance &msg, Eigen::Matrix4d &tf );
  
  private:
    std::shared_ptr<OctoObjectMap> octo_object_map_;
  };
}


#endif //SRC_OBJECT_SENSOR_MODEL_EVALUATOR_H
