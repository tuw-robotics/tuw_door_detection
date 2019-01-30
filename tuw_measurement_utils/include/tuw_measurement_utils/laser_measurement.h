//
// Created by felix on 20.01.19.
//

#ifndef TUW_LASER_MEASUREMENT_H
#define TUW_LASER_MEASUREMENT_H

#include <tuw_measurement_utils/measurement.h>
#include <tuw_measurement_utils/contour.h>
#include <sensor_msgs/LaserScan.h>

namespace tuw
{
  
  class LaserMeasurement : public Measurement
  {
  protected:
    sensor_msgs::LaserScan laser;
    std::vector<Contour::Beam> beams_;
  
  public:
    LaserMeasurement( const geometry_msgs::TransformStampedPtr _tf );
    
    ~LaserMeasurement() = default;
    
    void initFromScan( const sensor_msgs::LaserScan &_scan );
    
    const sensor_msgs::LaserScan &getLaser() const;
    
    void push_back( const Contour::Beam &_beam );
    
    //void resize( const size_t _sz );
    
    const size_t size() const
    {
      return beams_.size();
    }
    
    size_t size()
    {
      return beams_.size();
    }
    
    std::vector<Contour::Beam>::iterator begin();
    
    std::vector<Contour::Beam>::iterator end();
    
    const Contour::Beam &operator[]( const size_t _sz ) const;
    
    Contour::Beam &operator[]( const size_t _sz );
    
    void clear();
    
    double max_reading_;
    double min_reading_;
  };
  
}
#endif //PROJECT_LASER_MEASUREMENT_H
