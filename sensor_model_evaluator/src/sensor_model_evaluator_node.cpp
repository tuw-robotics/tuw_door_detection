//
// Created by felix on 01.03.19.
//
#include <sensor_model_evaluator_node.h>

using namespace tuw;

SensorModelEvaluatorNode::ParametersNode::ParametersNode( const ros::NodeHandle &nh )
{
  this->nh = nh;
  nh.param( std::string( "out_filename" ), filepath, std::string( "" ));
  nh.param( std::string( "laser_topic" ), laser_topic, std::string( "/r0/laser0/scan/raw" ));
  nh.param( std::string( "map_topic" ), map_topic, std::string( "/map" ));
  nh.param( std::string( "estimate_model_parameters" ), estimate_parameters, false );
}

SensorModelEvaluatorNode::SensorModelEvaluatorNode( ros::NodeHandle &nh ) : evaluator_( nullptr ),
                                                                            tf_listener_( tf_buffer_ ),
                                                                            params_( ros::NodeHandle( "~" ))
{
  nh_ = nh;
  sub_laser_ = nh.subscribe( params_.laser_topic, 1000, &SensorModelEvaluatorNode::callbackLaser, this );
  sub_map_ = nh.subscribe( params_.map_topic, 1, &SensorModelEvaluatorNode::callbackMap, this );
  sub_dummy_ = nh.subscribe( "dummy_msg", 1, &SensorModelEvaluatorNode::callbackDummy, this );
  pub_map_eth_ = nh.advertise<grid_map_msgs::GridMap>( "/grid_map", 4 );
  
  ROS_INFO( "subscribed to %s\n%s\n", params_.laser_topic.c_str(), params_.map_topic.c_str());
  f_callback = boost::bind( &SensorModelEvaluatorNode::reconfigureCallback, this, _1, _2 );
  server.setCallback( f_callback );
}

void SensorModelEvaluatorNode::callbackObjectDetection( const tuw_object_msgs::ObjectDetectionConstPtr &obj )
{
  Eigen::Matrix4d tf;
  if ( tryPoseFetch( tf, "map", obj->header.frame_id ))
  {
    //
  }
}

bool SensorModelEvaluatorNode::tryPoseFetch( Eigen::Matrix4d &tf_w_base, const std::string &world_frame,
                                             const std::string &target_frame )
{
  //TODO: parameters
  try
  {
    
    geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
        world_frame, target_frame, ros::Time( 0 ));
    
    geometry_msgs::TransformStampedPtr stampedPtr;
    stampedPtr.reset( new geometry_msgs::TransformStamped( stamped_tf ));
    
    Measurement::transformStamped2Eigen( stampedPtr, tf_w_base );
    
  } catch (tf2::TransformException &ex)
  {
    
    ROS_INFO( "DoorDetectorNode::getStaticTF" );
    ROS_ERROR( "%s", ex.what());
    return false;
    
  }
  
  return true;
}

void SensorModelEvaluatorNode::callbackDummy( const std_msgs::String &msg )
{
  estimationLoop();
}

void SensorModelEvaluatorNode::reconfigureCallback(
    const sensor_model_evaluator::SensorModelEvaluatorNodeConfig &callback, uint32_t level )
{
  config_ = callback;
  if ( evaluator_ )
  {
    evaluator_->configure( callback );
  }
}

void SensorModelEvaluatorNode::callbackMap( const nav_msgs::OccupancyGridConstPtr &map )
{
  if ( !evaluator_ )
  {
    evaluator_.reset( new LaserSensorModelEvaluator( map ));
    evaluator_->configure( config_ );
  }
}

void SensorModelEvaluatorNode::callbackLaser( const sensor_msgs::LaserScan &_laser )
{
  
  if ( evaluator_ )
  {
    
    try
    {
      
      geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
          "map", _laser.header.frame_id, ros::Time( 0 ));
      
      geometry_msgs::TransformStampedPtr tf = boost::make_shared<geometry_msgs::TransformStamped>( stamped_tf );
      laser_measurement_.reset( new LaserMeasurement( tf ));
      laser_measurement_->initFromScan( _laser );
      
      evaluator_->evaluate( laser_measurement_ );
      
      auto &ranges = evaluator_->getRangesExpObs();
      SensorModelParameterEstimatorEM::ParametersEstimated estimation_params( 0, _laser.range_max,
                                                                              1.0 / _laser.range_max,
                                                                              0.15, 1.5, 1.0 );
      parameter_estimator_.setParams( estimation_params );
      for ( const auto inside : ranges )
      {
        parameter_estimator_.add( inside.second.first, inside.second.second );
      }
      
    } catch (tf2::TransformException &ex)
    {
      ROS_WARN( "transform lookup failed SensorModelEvaluatorNode::callbackLaser\n" );
      ROS_WARN( "what(): %s\n", ex.what());
    }
    
  }
  laser_message_tick_ = ros::Time::now();
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
    if ( config_.serialize )
    {
      if ( params_.filepath != std::string( "" ))
      {
        evaluator_->serializeResult( params_.filepath );
      } else
      {
        ROS_WARN( "SensorModelEvaluatorNode::publish(): serialize option selected but no filepath given!" );
      }
    }
    
  }
}

void SensorModelEvaluatorNode::estimationLoop()
{
  printf( "Starting EM\n" );
  parameter_estimator_.compute();
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "sensor_model_evaluator_node" );
  
  ros::NodeHandle nh( "" );
  SensorModelEvaluatorNode eval_node( nh );
  ros::Rate r( 10 );
  
  while ( ros::ok())
  {
    
    ros::spinOnce();
    
    //eval_node.estimationLoop();
    
    eval_node.publish();
    
    r.sleep();
    
  }
  return 0;
}
