//
// Created by felix on 28.02.19.
//

#ifndef TUW_BEAM_H
#define TUW_BEAM_H

#include <tuw_geometry/point2d.h>
#include <Eigen/Core>

namespace tuw
{
  
  class Beam
  {
  public:
    //Beam()
    //{
    //};
    
    Beam();
    
    Beam( size_t global_idx, double range, double angle, Point2D end_point );
    
    //Beam(const Beam &) = delete;
    
    //Beam &operator=(const Beam &) = delete;
    
    ~Beam() = default;
    
    Point2D end_point;
    Point2D img_coords;
    Point2D img_base_coords;
    double range;
    double angle;
    size_t global_idx_;
    
    const bool is_valid() const;
    
    void set_valid( const bool v );
    
    const bool get_is_visible() const;
    
    void set_is_visible( const bool v );
    
    template<typename T>
    T get_direction_vector( const Eigen::Matrix4d &tf ) const;
    
    template<typename T>
    T transform(const Eigen::Matrix4d &tf) const;
    
    
    static std::shared_ptr<Beam> make_beam( size_t idx, double range, double angle, Point2D end_point );
  
  private:
    bool valid_beam;
    bool is_visible_;
  };
  
}
#endif //PROJECT_BEAM_H
