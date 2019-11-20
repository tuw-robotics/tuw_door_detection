//
// Created by felix on 28.02.19.
//

#include <tuw_measurement_utils/beam.h>

using namespace tuw;

Beam::Beam()
{
  valid_beam = false;
}

Beam::Beam( size_t global_idx, double _range, double _angle, Point2D _end_point )
{
  global_idx_ = global_idx;
  range = _range;
  angle = _angle;
  end_point = _end_point;
  valid_beam = true;
}

const bool Beam::is_valid() const
{
  return valid_beam;
}

void Beam::set_valid( const bool v )
{
  valid_beam = v;
}

void Beam::set_is_visible( const bool v )
{
  is_visible_ = v;
}

const bool Beam::get_is_visible() const
{
  return is_visible_;
}

const size_t &Beam::global_idx() const
{
  return global_idx_;
}

std::shared_ptr<Beam> Beam::make_beam( size_t global_idx, double range, double angle, Point2D end_point )
{
  return std::make_shared<Beam>( global_idx, range, angle, end_point );
}

//template <>
//Eigen::Vector2d Beam::transform( const Eigen::Matrix4d &tf ) const
//{
//  Eigen::Vector4d p = tf * Eigen::Vector4d(end_point.x(), end_point.y(), 0, 1);
//  p = p / p[3];
//  return p.head<2>();
//}
//
//template <>
//cv::Point2d Beam::transform( const Eigen::Matrix4d &tf ) const
//{
//  auto p = transform<Eigen::Vector2d>(tf);
//  return cv::Point2d(p.x(), p.y());
//}
//
//template <>
//Point2D Beam::transform( const Eigen::Matrix4d &tf ) const
//{
//  auto p = transform<Eigen::Vector2d>(tf);
//  return Point2D(p.x(), p.y());
//}
