//
// Created by felix on 11.04.19.
//

#ifndef TUW_DOOR_DETECTION_OBJECT_SENSOR_MODEL_EVALUATOR_H
#define TUW_DOOR_DETECTION_OBJECT_SENSOR_MODEL_EVALUATOR_H

#include <tuw_object_msgs/ObjectDetection.h>
#include <datastructures/octo_object_map.h>
#include <Eigen/Core>
#include <map>
#include <memory>

namespace tuw
{
  class ObjectSensorModel
  {
  public:
    
    using Ptr = std::shared_ptr<ObjectSensorModel>;
    
    struct Result {
      Eigen::Vector3d exp;
      Eigen::Vector3d obs;
      float dist;
      float angular_dist;
    };
    
    ObjectSensorModel();
    
    ~ObjectSensorModel() = default;
    
    /**
     * If msg type is SHAPE_MAP_DOOR then the doors location is stored in a pcl octree.
     * If msg type is SHAPE_DOOR the the msg is treated as an actual observation and its nearest neighbor in the door map is searched
     *
     * @param msg Object message, in particular SHAPE_MAP_DOOR and SHAPE_DOOR are supported
     * @param tf The tf from laser space into world space
     */
    void process( const tuw_object_msgs::ObjectWithCovariance &msg, const Eigen::Matrix4d &tf );
    
    /**
     * Evaluates the processed messages i.e. distance euclidean and angular computation
     */
    void evaluate();
    
    void serializeResult(const std::string &filename);
  
  private:
    std::shared_ptr<OctoObjectMap> octo_object_map_;
    
    /**
     * contains the observed and expected measurements collected during a @see ObjectSensorModel::process call.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> measurements_table_;
    
    /**
     * contains the pose of the laser in worldspace coordinates (make sure localization is running)
     * for each entry in @see ObjectSensorModel::measurements_table_;
     */
    std::vector<Eigen::Matrix4d> laser_pose_worldspace_;
    
    /**
     * contains the results from an evaluate call
     */
    std::vector<Result> results_;
    
    unsigned int missed_counter_;
    unsigned int observation_counter_;
  };
  
  using ObjectSensorModelPtr = std::shared_ptr<ObjectSensorModel>;
}


#endif //SRC_OBJECT_SENSOR_MODEL_EVALUATOR_H
