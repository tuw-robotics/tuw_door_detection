//
// Created by felix on 20.01.19.
//

#ifndef TUW_LASER_MEASUREMENT_H
#define TUW_LASER_MEASUREMENT_H

#include <tuw_measurement_utils/measurement.h>
#include <tuw_measurement_utils/contour.h>
#include <tuw_measurement_utils/beam.h>
#include <sensor_msgs/LaserScan.h>

namespace tuw
{
  
  class LaserMeasurement : public Measurement
  {
  protected:
    sensor_msgs::LaserScan laser;
    std::vector<Beam> beams_;
  
  public:
    LaserMeasurement( const geometry_msgs::TransformStampedPtr _tf );
    
    ~LaserMeasurement() = default;
    
    void initFromScan( const sensor_msgs::LaserScan &_scan );
    
    const sensor_msgs::LaserScan &getLaser() const;
    
    void push_back( const Beam &_beam );
    
    //void resize( const size_t _sz );
    
    const size_t size() const
    {
      return beams_.size();
    }
    
    size_t size()
    {
      return beams_.size();
    }
    
    std::vector<Beam>::iterator begin();
    
    std::vector<Beam>::iterator end();
    
    const Beam &operator[]( const size_t _sz ) const;
    
    Beam &operator[]( const size_t _sz );
    
    void clear();
    
    double max_reading_;
    double min_reading_;
  };
  
  using LaserMeasurementPtr = std::shared_ptr<LaserMeasurement>;
}
#endif //PROJECT_LASER_MEASUREMENT_H
