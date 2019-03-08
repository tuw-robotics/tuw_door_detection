//
// Created by felix on 08.03.19.
//

#ifndef TUW_TRANSFORMS_H
#define TUW_TRANSFORMS_H

#include <geometry_msgs/Pose.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tuw
{
  
  template <class Derived>
  void rot2Quat( Eigen::DenseBase<Derived> &m, Eigen::Quaternion<Derived> &q )
  {
    q( m.block( 0, 0, 3, 3 ));
  }
  
  template <typename T, class Derived>
  void rot2Quat( cv::Mat_<T> &m, Eigen::Quaternion<Derived> &q )
  {
    cv::Mat_<T> rot = m.at( cv::Range( 0, 3 ), cv::Range( 0, 3 ));
    Eigen::DenseBase<Derived> m2q;
    for ( int r = 0; r < rot.rows; ++r )
    {
      for ( int c = 0; c < rot.cols; ++c )
      {
        m2q( r, c ) = rot.at( r, c );
      }
    }
    
    rot2Quat(m2q, q);
  }
  
  template <class Derived>
  void rot2Quat( Eigen::DenseBase<Derived> &m, geometry_msgs::Pose::_orientation_type &q )
  {
    Eigen::Quaternion<Derived> q_interm;
    rot2Quat( m, q_interm );
    
    q.x = q_interm.x();
    q.y = q_interm.y();
    q.z = q_interm.z();
    q.w = q_interm.w();
  }
  
  template <class Derived>
  void convert( Eigen::DenseBase<Derived> &src, geometry_msgs::Pose &des )
  {
    Eigen::DenseBase<Derived> m = src.eval();
    des.position.x = m( 0, 3 );
    des.position.y = m( 1, 3 );
    des.position.z = m( 2, 3 );
    rot2Quat( m, des.orientation );
  }
  
  template <typename T>
  void convert( cv::Mat_<T> &src, geometry_msgs::Pose &des )
  {
    assert( src.rows == 4 && src.cols == 4 );
    
    des.position.x = src.at( 0, 3 );
    des.position.y = src.at( 1, 3 );
    des.position.z = src.at( 2, 3 );
    
    rot2Quat( src, des.orientation );
    
  }
  
};

#endif //PROJECT_TRANSFORMS_H
