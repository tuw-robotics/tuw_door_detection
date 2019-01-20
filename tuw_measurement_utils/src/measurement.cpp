//
// Created by felix on 13.11.18.
//

#include "tuw_measurement_utils/measurement.h"
#include <tuw_geometry/utils.h>

using namespace tuw;

void
Measurement::transformStamped2Eigen( const geometry_msgs::TransformStampedPtr tf, Eigen::Matrix<double, 4, 4> &mat )
{
  auto ros_q = tf->transform.rotation;
  auto ros_t = tf->transform.translation;

  Eigen::Quaterniond q;
  q.x() = ros_q.x;
  q.y() = ros_q.y;
  q.z() = ros_q.z;
  q.w() = ros_q.w;

  Eigen::Vector3d t(ros_t.x, ros_t.y, ros_t.z);

  mat.setIdentity();
  mat.topLeftCorner<3, 3>() = q.toRotationMatrix();
  mat.topRightCorner<3, 1>() = t;
}

void Measurement::cv2Eigen( const cv::Matx44d &_we_dont_want, Eigen::Matrix<double, 4, 4> &_we_want )
{
  for ( int i = 0; i < _we_dont_want.rows; ++i )
  {
    for ( int j = 0; j < _we_dont_want.cols; ++j )
    {
      _we_want(i, j) = _we_dont_want(i, j);
    }
  }
}

Measurement::Measurement( const Eigen::Matrix<double, 4, 4> &tfWorldSensor ) : tfWorldSensor(tfWorldSensor)
{
}

Measurement::Measurement( const geometry_msgs::TransformStampedPtr _tf )
{
  setTfWorldSensor(_tf);
}

const Eigen::Matrix<double, 4, 4> &Measurement::getTfWorldSensor() const
{
  return tfWorldSensor;
}

void Measurement::setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor, bool override_stamped )
{
  Measurement::tfWorldSensor = tfWorldSensor;
  if ( override_stamped )
  {
    Eigen::Matrix3d r = tfWorldSensor.topLeftCorner<3, 3>();
    Eigen::Quaterniond q(r);
    Eigen::Vector3d t = tfWorldSensor.topRightCorner<3, 1>();

    stamped_tf_->transform.translation.x = t(0);
    stamped_tf_->transform.translation.y = t(1);
    stamped_tf_->transform.translation.z = t(2);

    stamped_tf_->transform.rotation.x = q.x();
    stamped_tf_->transform.rotation.y = q.y();
    stamped_tf_->transform.rotation.z = q.z();
    stamped_tf_->transform.rotation.w = q.w();

  }
}

void Measurement::setTfWorldSensor( cv::Mat &_tf )
{
  cv2Eigen(_tf, tfWorldSensor);
}

const geometry_msgs::TransformStampedPtr Measurement::getStampedTf() const
{
  return stamped_tf_;
}

void Measurement::setTfWorldSensor( geometry_msgs::TransformStampedPtr _tf )
{
  stamped_tf_.reset(new geometry_msgs::TransformStamped(*_tf));
  transformStamped2Eigen(stamped_tf_, tfWorldSensor);
}



