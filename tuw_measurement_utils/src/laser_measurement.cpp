//
// Created by felix on 20.01.19.
//

#include <tuw_measurement_utils/laser_measurement.h>

using namespace tuw;

LaserMeasurement::LaserMeasurement( const geometry_msgs::TransformStampedPtr _tf ) : Measurement( _tf )
{
}

void LaserMeasurement::clear()
{
  beams_.clear();
}

void LaserMeasurement::initFromScan( const sensor_msgs::LaserScan &_scan )
{
  clear();
  this->laser = _scan;
  
  max_reading_ = -std::numeric_limits<double>::max();
  min_reading_ = std::numeric_limits<double>::max();
  
  std::size_t n = laser.ranges.size();
  std::size_t ii = 0;
  
  for ( ; ii < n; ++ii )
  {
    double range = laser.ranges[ii];
    if ( isfinite( range ) && range < laser.range_max )
    {
      
      const double angle = laser.angle_min + (laser.angle_increment * ii);
      const Point2D pt( cos( angle ) * range, sin( angle ) * range );
      push_back( Beam( ii, range, angle, pt ));
      
      max_reading_ = std::max( max_reading_, range );
      min_reading_ = std::min( min_reading_, range );
    }
  }
}

const sensor_msgs::LaserScan &LaserMeasurement::getLaser() const
{
  return laser;
}

void LaserMeasurement::push_back( const tuw::Beam &_beam )
{
  this->beams_.push_back( _beam );
}

//void LaserMeasurement::resize( const size_t _sz )
//{
//  beams_.resize( _sz );
//}

const Beam &LaserMeasurement::operator[]( const size_t _sz ) const
{
  return beams_[_sz];
}

Beam &LaserMeasurement::operator[]( const size_t _sz )
{
  return beams_[_sz];
}

std::vector<Beam>::iterator LaserMeasurement::begin()
{
  return beams_.begin();
}

std::vector<Beam>::iterator LaserMeasurement::end()
{
  return beams_.end();
}
