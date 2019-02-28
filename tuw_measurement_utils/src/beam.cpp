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

template <>
Eigen::Vector4d Beam::get_direction_vector<Eigen::Vector4d>( const Eigen::Matrix4d &base ) const
{
  Eigen::Vector4d beam_end = Eigen::Vector4d( end_point.x(), end_point.y(), 0, 1 );
  Eigen::Vector4d w_p_mp = base * beam_end;
  w_p_mp = w_p_mp / w_p_mp[3];
  w_p_mp.normalize();
  return std::move( w_p_mp );
}

template <>
Eigen::Vector2d Beam::get_direction_vector<Eigen::Vector2d>( const Eigen::Matrix4d &tf ) const
{
  Eigen::Vector4d ev = get_direction_vector < Eigen::Vector4d > (tf);
  return std::move( Eigen::Vector2d( ev.x(), ev.y()));
}

template <>
Eigen::Vector3d Beam::get_direction_vector( const Eigen::Matrix4d &tf ) const
{
  Eigen::Vector4d ev = get_direction_vector < Eigen::Vector4d > (tf);
  return std::move( Eigen::Vector3d( ev.x(), ev.y(), ev.z()));
}

template <>
cv::Point2d Beam::get_direction_vector( const Eigen::Matrix4d &tf ) const
{
  auto ev = get_direction_vector < Eigen::Vector2d > (tf);
  return cv::Point2d( ev.x(), ev.y());
}

template <>
Point2D Beam::get_direction_vector( const Eigen::Matrix4d &tf ) const
{
  auto ev = get_direction_vector < cv::Point2d > (tf);
  return Point2D( ev );
}

std::shared_ptr<Beam> Beam::make_beam( size_t global_idx, double range, double angle, Point2D end_point )
{
  return std::make_shared<Beam>( global_idx, range, angle, end_point );
}