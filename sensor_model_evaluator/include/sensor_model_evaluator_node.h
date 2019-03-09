//
// Created by felix on 01.03.19.
//

#ifndef TUW_SENSORMODELEVALUATORNODE_H
#define TUW_SENSORMODELEVALUATORNODE_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_measurement_utils/laser_measurement.h>
#include <sensor_model_evaluator.h>
#include <nav_msgs/GetMap.h>

namespace tuw
{

  class SensorModelEvaluatorNode
  {
  public:
    SensorModelEvaluatorNode( ros::NodeHandle &nh );

    void callbackLaser( const sensor_msgs::LaserScan &laser );

    void callbackMap( const nav_msgs::OccupancyGridConstPtr &map );

    void publish();

  private:

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_map_eth_;
    ros::NodeHandle nh_;

    std::string filepath_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::OccupancyGridConstPtr map_;

    LaserMeasurementPtr laser_measurement_;
    SensorModelEvaluatorPtr evaluator_;

  };

};
#endif //PROJECT_SENSORMODELEVALUATORNODE_H
