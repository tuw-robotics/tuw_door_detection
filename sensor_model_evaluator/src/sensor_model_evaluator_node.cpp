//
// Created by felix on 01.03.19.
//
#include <sensor_model_evaluator_node.h>

using namespace tuw;

SensorModelEvaluatorNode::SensorModelEvaluatorNode( ros::NodeHandle &nh ) : evaluator_(nullptr),
                                                                            tf_listener_(tf_buffer_)
{
  nh.param(std::string("out_filename"), filepath_, std::string(""));

  nh_ = nh;
  sub_laser_ = nh.subscribe("/r0/laser0/scan/raw", 1000, &SensorModelEvaluatorNode::callbackLaser, this);
  sub_map_ = nh.subscribe("/map", 1, &SensorModelEvaluatorNode::callbackMap, this);
  pub_map_eth_ = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 4);
  f_callback = boost::bind(&SensorModelEvaluatorNode::reconfigureCallback, this, _1, _2);
  server.setCallback(f_callback);
}

void SensorModelEvaluatorNode::reconfigureCallback(
    const sensor_model_evaluator::SensorModelEvaluatorNodeConfig &callback, uint32_t level )
{
  config_ = callback;
  if ( evaluator_ )
  {
    evaluator_->configure(callback);
  }
}

void SensorModelEvaluatorNode::callbackMap( const nav_msgs::OccupancyGridConstPtr &map )
{
  if ( !evaluator_ )
  {
    evaluator_.reset(new SensorModelEvaluator(map));
    evaluator_->configure(config_);
  }
}

void SensorModelEvaluatorNode::callbackLaser( const sensor_msgs::LaserScan &_laser )
{

  if ( evaluator_ )
  {

    try
    {

      geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
          "map", "r0/laser0", ros::Time(0));

      geometry_msgs::TransformStampedPtr tf = boost::make_shared<geometry_msgs::TransformStamped>(stamped_tf);
      laser_measurement_.reset(new LaserMeasurement(tf));
      laser_measurement_->initFromScan(_laser);

      evaluator_->evaluate(laser_measurement_);

    } catch ( tf2::TransformException &ex )
    {
      ROS_WARN("transform lookup failed SensorModelEvaluatorNode::callbackLaser\n");
      ROS_WARN("what(): %s\n", ex.what());
    }

  }

}

void SensorModelEvaluatorNode::publish()
{
  if ( evaluator_ )
  {
    //if ( evaluator_->getMap( pub_map ))
    //{
    //  pub_map_eth_.publish( pub_map );
    //} else
    //{
    //  ROS_WARN( "publish called but no result available" );
    //}
    if ( filepath_ != std::string("") && config_.serialize )
    {
      evaluator_->serializeResult(filepath_);
    }
  }
}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "sensor_model_evaluator_node");

  ros::NodeHandle nh("~");
  SensorModelEvaluatorNode eval_node(nh);
  ros::Rate r(10);

  while ( ros::ok())
  {

    ros::spinOnce();

    eval_node.publish();

    r.sleep();

  }
  return 0;
}