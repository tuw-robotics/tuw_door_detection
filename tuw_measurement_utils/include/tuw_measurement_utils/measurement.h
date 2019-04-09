#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <boost/shared_ptr.hpp>

namespace tuw
{

  class Measurement
  {
  public:

    explicit Measurement( const Eigen::Matrix<double, 4, 4> &tfWorldSensor );

    explicit Measurement( const geometry_msgs::TransformStampedPtr _tf );

    ~Measurement() = default;

    const Eigen::Matrix<double, 4, 4> &getTfWorldSensor() const;

    void setTfWorldSensor( cv::Mat &_tf );

    void setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor, bool override_stamped = false );

    void setTfWorldSensor( const geometry_msgs::TransformStampedPtr _tf );

    const geometry_msgs::TransformStampedPtr getStampedTf() const;

    void cv2Eigen( const cv::Matx44d &, Eigen::Matrix<double, 4, 4> & );
  
    static void transformStamped2Eigen( const geometry_msgs::TransformStampedPtr &, Eigen::Matrix<double, 4, 4> & );
    
  protected:
    Eigen::Matrix<double, 4, 4> tfWorldSensor;
    geometry_msgs::TransformStampedPtr stamped_tf_;
  };


};

#endif
