//
// Created by felix on 27.03.19.
//

int main( int argc, char **argv )
{
  ros::init( argc, argv, "sensor_model_evaluator_node" );
  
  ros::NodeHandle nh( "" );
  SensorModelParameterEstimatorEMNode eval_node( nh );
  ros::Rate r( 10 );
  
  while ( ros::ok())
  {
    
    ros::spinOnce();
    
    eval_node.publish();
    
    r.sleep();
    
  }
  return 0;
}