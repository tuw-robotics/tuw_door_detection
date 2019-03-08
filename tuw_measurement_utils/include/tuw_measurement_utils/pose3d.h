//
// Created by felix on 08.03.19.
//

#ifndef TUW_POSE3D_H
#define TUW_POSE3D_H

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <transforms.h>


namespace tuw
{
  class Pose3D
  {
  public:
    template <typename T>
    Pose3D( Eigen::DenseBase<T> &m )
    {
    
    }
    
    template <typename T>
    Pose3D( cv::Mat_<T> &m )
    {
    
    }
    
    double distance3DTo(const Pose3D &other)

  private:
    Eigen::Vector3<T> origin;
    Eigen::Quaternion<T> rotation;
  };
};


#endif //PROJECT_POSE3D_H
