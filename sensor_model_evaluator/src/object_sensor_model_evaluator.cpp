//
// Created by felix on 11.04.19.
//

#include <object_sensor_model_evaluator.h>

using namespace tuw;

ObjectSensorModel::ObjectSensorModel()
{
  octo_object_map_ = std::make_shared<OctoObjectMap>( 0.5f );
}

void ObjectSensorModel::process( tuw_object_msgs::ObjectWithCovariance &obj, Eigen::Matrix4d &tf )
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
    if ( !octo_object_map_->searchBestPCL( search_pnt, 0.5, out_pnt, true ))
    {
      pcl::PointXYZ pcl_pnt( tworld.x(), tworld.y(), tworld.z());
      octo_object_map_->insert( pcl_pnt );
    }
  } else if ( obj.object.shape == Object::SHAPE_DOOR ) //Crosscheck with map
  {
    if ( !octo_object_map_->searchBestPCL( search_pnt, 0.5, out_pnt, true ))
    {
      //miss
    } else
    {
      //hit
    }
  }
}

