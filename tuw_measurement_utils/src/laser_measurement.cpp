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

const sensor_msgs::LaserScan LaserMeasurement::filterMessage( const std::vector<std::pair<std::size_t, std::size_t>> &ranges )
{
  //Flatten the range (get rid of overlaps)
  std::set<int> exclude;
  for ( const std::pair<std::size_t, std::size_t> &range_idx : ranges )
  {
    for ( int i = range_idx.first; i <= range_idx.second; ++i )
    {
      exclude.insert( i );
    }
  }
  
  sensor_msgs::LaserScan scan_filtered;
  scan_filtered.header = this->laser.header;
  scan_filtered.range_max = this->laser.range_max;
  scan_filtered.range_min = this->laser.range_min;
  scan_filtered.angle_increment = this->laser.angle_increment;
  scan_filtered.angle_max = this->laser.angle_max;
  scan_filtered.angle_min = this->laser.angle_min;
  scan_filtered.scan_time = this->laser.scan_time;
  scan_filtered.time_increment = this->laser.time_increment;
  
  scan_filtered.ranges.resize(this->laser.ranges.size());
  scan_filtered.intensities.resize(this->laser.intensities.size());
  std::copy(this->laser.ranges.begin(), this->laser.ranges.end(), scan_filtered.ranges.begin());
  std::copy(this->laser.intensities.begin(), this->laser.intensities.end(), scan_filtered.intensities.begin());
  
  //Copy the scan without doors
  printf("filtering %ld/%ld\n", exclude.size(), scan_filtered.ranges.size());
  for (int i = 0; i < scan_filtered.ranges.size(); ++i)
  {
    if (exclude.count(i) != 0)
    {
      scan_filtered.ranges[i] = std::nanf("");
      scan_filtered.intensities[i] = std::nanf("");
    }
  }
  
  return std::move(scan_filtered);
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
