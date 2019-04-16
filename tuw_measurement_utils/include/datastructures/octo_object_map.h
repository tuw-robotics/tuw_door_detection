//
// Created by felix on 10.04.19.
//

#ifndef TUW_DOOR_DETECTION_OCTO_OBJECT_MAP_H
#define TUW_DOOR_DETECTION_OCTO_OBJECT_MAP_H

#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <map>

namespace tuw
{
  class OctoMapNode
  {
  
  public:
    OctoMapNode( unsigned int idx, pcl::PointXYZ &pnt );
    
    ~OctoMapNode();
    
    std::size_t &idx();
    
    Eigen::Vector3d &pose();
    
    unsigned int &seen();
    
    ros::Time &last_seen();
    
    ros::Duration toc();
  
  private:
    ros::Time last_seen_;
    unsigned int seen_;
    std::size_t idx_;
    Eigen::Vector3d pose_;
  };
  
  class OctoObjectMap
  {
  public:
    //OctoObjectMap();
    
    OctoObjectMap(float resolution);
    
    ~OctoObjectMap();
    
    pcl::PointCloud<pcl::PointXYZ> &getPCloud();
    
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &getOctree();
    
    template <typename T>
    bool searchBestPCL( pcl::PointXYZ &search_point, float radius, T &outPoint, float &distance_squared, bool auto_increment = true )
    {
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      
      if ( pcl_octree_->radiusSearch( search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
      {
        size_t best_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        
        for ( size_t i = 0; i < pointRadiusSquaredDistance.size(); ++i )
        {
          float sq_dist = pointRadiusSquaredDistance[i];
          if ( sq_dist < best_dist )
          {
            best_idx = i;
            best_dist = sq_dist;
          }
        }
        
        if ( auto_increment )
        {
          auto obj_stored = obj_nodes_.find( pointIdxRadiusSearch[best_idx] );
          if ( obj_stored != obj_nodes_.end())
          {
            obj_stored->second.seen()++;
            obj_stored->second.last_seen() = ros::Time::now();
          }
        }
        
        outPoint.x() = pcl_objcloud->points[pointIdxRadiusSearch[best_idx]].x;
        outPoint.y() = pcl_objcloud->points[pointIdxRadiusSearch[best_idx]].y;
        outPoint.z() = pcl_objcloud->points[pointIdxRadiusSearch[best_idx]].z;
        distance_squared = pointRadiusSquaredDistance[pointRadiusSquaredDistance[best_idx]];
        return true;
      }
      return false;
    }
    
    void insert( pcl::PointXYZ &pcl_pnt );
    
    std::map<unsigned int, OctoMapNode>::iterator nodes_begin();
    
    std::map<unsigned int, OctoMapNode>::iterator nodes_end();
    
    std::string print();
  
  private:
    std::map<unsigned int, OctoMapNode> obj_nodes_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_objcloud;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr pcl_octree_;
    
  };
  
  
};


#endif //SRC_OCTO_OBJECT_MAP_H
