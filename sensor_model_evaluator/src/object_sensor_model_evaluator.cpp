//
// Created by felix on 11.04.19.
//

#include <object_sensor_model_evaluator.h>

using namespace tuw;

ObjectSensorModel::ObjectSensorModel()
{
  octo_object_map_ = std::make_shared<OctoObjectMap>( 0.5f );
  observation_counter_ = 0;
  missed_counter_ = 0;
}

void ObjectSensorModel::evaluate()
{
  size_t idx = 0;
  for ( auto obs_exp : measurements_table_ )
  {
    auto &observation = obs_exp.first;
    auto &expectation = obs_exp.second;
    auto tf_world_laser = laser_pose_worldspace_[idx++];
    Eigen::Vector3d pnt_world_laser = tf_world_laser.topRightCorner<3, 1>();
    float dist_sqrt = (observation - expectation).norm();
    
    Eigen::Vector3d viewing_dir_obs = observation - pnt_world_laser;
    Eigen::Vector3d viewing_dir_exp = expectation - pnt_world_laser;
    viewing_dir_exp.normalize();
    viewing_dir_obs.normalize();
    float dist_angle = acos( viewing_dir_obs.dot( viewing_dir_exp ));
    
    Result res;
    res.dist = dist_sqrt;
    res.angular_dist = dist_angle;
    res.exp = expectation;
    res.obs = observation;
    results_.push_back( std::move( res ));
  }
  std::cout << "hit/miss ratio: " << (observation_counter_ - missed_counter_) << "/" << missed_counter_ << std::endl;
  std::cout << "total: " << observation_counter_ << std::endl;
}

void ObjectSensorModel::clear()
{
  results_.clear();
  measurements_table_.clear();
  laser_pose_worldspace_.clear();
  missed_counter_ = 0;
  observation_counter_ = 0;
}

void ObjectSensorModel::process( const tuw_object_msgs::ObjectWithCovariance &obj, const Eigen::Matrix4d &tf )
{
  using tuw_object_msgs::ObjectDetection;
  using tuw_object_msgs::Object;
  
  auto &door_obj = obj.object;
  auto &position = door_obj.pose.position;
  auto &orientation = door_obj.pose.orientation;
  
  Eigen::Matrix4d pose_mat_w;
  Eigen::Quaterniond qobject( orientation.w, orientation.x, orientation.y, orientation.z );
  
  pose_mat_w.topLeftCorner<3, 3>() = qobject.toRotationMatrix();
  pose_mat_w.topRightCorner<3, 1>() = Eigen::Vector3d( position.x, position.y, position.z );
  pose_mat_w = tf * pose_mat_w;
  
  Eigen::Quaterniond qworld( pose_mat_w.topLeftCorner<3, 3>());
  Eigen::Vector3d tworld( pose_mat_w.topRightCorner<3, 1>());
  
  pcl::PointXYZ search_pnt( tworld.x(), tworld.y(), tworld.z());
  Eigen::Vector3d out_pnt;
  
  if ( obj.object.shape == Object::SHAPE_MAP_DOOR ) //Door is in map -> store it
  {
    float dist_squared = -1;
    if ( !octo_object_map_->searchBestPCL( search_pnt, 0.5, out_pnt, dist_squared, true ))
    {
      pcl::PointXYZ pcl_pnt( tworld.x(), tworld.y(), tworld.z());
      octo_object_map_->insert( pcl_pnt );
    }
  } else if ( obj.object.shape == Object::SHAPE_DOOR ) //Crosscheck with map
  {
    observation_counter_++;
    float dist_squared = -1;
    if ( !octo_object_map_->searchBestPCL( search_pnt, 0.5, out_pnt, dist_squared, true ))
    {
      missed_counter_++;
    } else
    {
      laser_pose_worldspace_.push_back( tf );
      measurements_table_.push_back( std::move( std::make_pair( tworld, out_pnt )));
    }
  }
}

void ObjectSensorModel::internal_serializeResult( boost::filesystem::ofstream &of )
{
  for ( Result &res : results_ )
  {
    of << res.dist << ", " << res.angular_dist << "\n";
  }
  //of << std::endl;
  of.flush();
  of.close();
}

void ObjectSensorModel::serializeResult( const std::string &filepath, const bool continuous )
{
  using boost::filesystem::path;
  path p( filepath );
  if ( boost::filesystem::exists( p ))
  {
    if ( continuous )
    {
      boost::filesystem::ofstream of( p, std::ios::app | std::ios::ate );
      internal_serializeResult( of );
    } else
    {
      boost::filesystem::ofstream of( p, std::ios::out );
      internal_serializeResult( of );
    }
    //if ( continuous_outstream_ )
    //{
    //  boost::filesystem::ofstream of( p, std::ios::app | std::ios::ate );
    //  internalSerialize( of );
    //  //of << std::endl;
    //  of.close();
    //} else if ( filesys_force_override_ )
    //{
    //  boost::filesystem::ofstream of( p, std::ios::out );
    //  internalSerialize( of );
    //  of.close();
    //}
  } else
  {
    boost::filesystem::ofstream of( p, std::ios::out );
    internal_serializeResult(of);
  }
}
