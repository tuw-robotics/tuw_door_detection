//
// Created by felix on 10.04.19.
//

#include <datastructures/octo_object_map.h>
#include <iostream>
#include <sstream>

using namespace tuw;

OctoMapNode::OctoMapNode( unsigned int idx, pcl::PointXYZ &pnt )
{
  idx_ = idx;
  pose_ = Eigen::Vector3d( pnt.x, pnt.y, pnt.z );
  seen_ = 1;
  last_seen_ = ros::Time::now();
}

OctoMapNode::~OctoMapNode() = default;

std::size_t &OctoMapNode::idx()
{
  return idx_;
}

Eigen::Vector3d &OctoMapNode::pose()
{
  return pose_;
}

unsigned int &OctoMapNode::seen()
{
  return seen_;
}

ros::Time &OctoMapNode::last_seen()
{
  return last_seen_;
}

ros::Duration OctoMapNode::toc()
{
  return ros::Time::now() - last_seen_;
}



//OctoObjectMap::OctoObjectMap() : pcl_objcloud( new pcl::PointCloud<pcl::PointXYZ>()),
//                                 pcl_octree_( new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>( 0.5f ))
//{
//  pcl_octree_->setInputCloud( pcl_objcloud );
//  pcl_octree_->addPointsFromInputCloud();
//}

OctoObjectMap::OctoObjectMap( float resolution ) : pcl_objcloud( new pcl::PointCloud<pcl::PointXYZ>()),
                                                   pcl_octree_( new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(
                                                       resolution ))
{
  pcl_octree_->setInputCloud( pcl_objcloud );
  pcl_octree_->addPointsFromInputCloud();
}

OctoObjectMap::~OctoObjectMap()
{
}

void OctoObjectMap::insert( pcl::PointXYZ &pcl_pnt )
{
  pcl_octree_->addPointToCloud( pcl_pnt, pcl_objcloud );
  unsigned int idx = pcl_objcloud->size() - 1;
  auto node = OctoMapNode( idx, pcl_pnt );
  obj_nodes_.insert( std::make_pair( idx, node ));
}

std::map<unsigned int, OctoMapNode>::iterator OctoObjectMap::nodes_begin()
{
  return obj_nodes_.begin();
}

std::map<unsigned int, OctoMapNode>::iterator OctoObjectMap::nodes_end()
{
  return obj_nodes_.end();
}

std::string OctoObjectMap::print()
{
  std::stringstream sstr;
  sstr << "pcl doors in map: " << pcl_objcloud->size() << std::endl;
  for ( int i = 0; i < pcl_objcloud->size(); ++i )
  {
    auto fm_it = obj_nodes_.find( i );
    if ( fm_it != obj_nodes_.end())
    {
      sstr << fm_it->first << ":\t " << "seen: " << fm_it->second.seen() << "\t timestamp "
           << fm_it->second.toc().sec << "s" << std::endl;
    }
  }
  std::cout << sstr.str() << std::endl;
  return sstr.str();
}


