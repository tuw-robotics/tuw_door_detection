//
// Created by felix on 01.03.19.
//

#ifndef TUW_SENSORMODELEVALUATORNODE_H
#define TUW_SENSORMODELEVALUATORNODE_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/GetMap.h>

#include <sensor_msgs/LaserScan.h>
#include <tuw_measurement_utils/laser_measurement.h>
#include <sensor_model_evaluator.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_model_evaluator/SensorModelEvaluatorNodeConfig.h>

namespace tuw
{

  class SensorModelEvaluatorNode
  {
  public:
    class ParametersNode
    {
    public:
      ParametersNode( const ros::NodeHandle &nh );
      ~ParametersNode() = default;
      
      ros::NodeHandle nh;
      std::string filepath;
      std::string laser_topic;
      std::string map_topic;
    };
    
    SensorModelEvaluatorNode( ros::NodeHandle &nh );

    void callbackLaser( const sensor_msgs::LaserScan &laser );

    void callbackMap( const nav_msgs::OccupancyGridConstPtr &map );

    void reconfigureCallback( const sensor_model_evaluator::SensorModelEvaluatorNodeConfig &callback, uint32_t level );

    void publish();

  private:

    dynamic_reconfigure::Server<sensor_model_evaluator::SensorModelEvaluatorNodeConfig> server;
    dynamic_reconfigure::Server<sensor_model_evaluator::SensorModelEvaluatorNodeConfig>::CallbackType f_callback;
    sensor_model_evaluator::SensorModelEvaluatorNodeConfig config_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_map_eth_;
    ros::NodeHandle nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::OccupancyGridConstPtr map_;

    LaserMeasurementPtr laser_measurement_;
    SensorModelEvaluatorPtr evaluator_;

    bool continuous_stream_;
    
    ParametersNode params_;

  };

};
#endif //PROJECT_SENSORMODELEVALUATORNODE_H
