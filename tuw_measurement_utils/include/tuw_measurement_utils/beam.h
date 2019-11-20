//
// Created by felix on 28.02.19.
//

#ifndef TUW_BEAM_H
#define TUW_BEAM_H

#include <tuw_geometry/point2d.h>
#include <Eigen/Core>

namespace tuw
{
  namespace transform_helpers {
    
    template<typename T>
    T transform_pnt(const Eigen::Matrix4d &tf, const Point2D &end_pnt)
    {
      Eigen::Vector4d p = tf * Eigen::Vector4d(end_pnt.x(), end_pnt.y(), 0, 1);
      p = p / p[3];
      return T(p.x(), p.y());
      //return p.head<2>();
    }
    
    //template <typename T>
    //T fwd_vector(const Eigen::Vector4d &v);
    
    template<typename T, typename std::enable_if<std::is_base_of<Eigen::Vector2d, T>::value>::type* = nullptr>
    Eigen::Vector2d fwd_vector( const Eigen::Vector4d &v )
    {
      return v.head<2>();
    }
    
    template<typename T, typename std::enable_if<std::is_base_of<Eigen::Vector3d, T>::value>::type* = nullptr>
    Eigen::Vector3d fwd_vector( const Eigen::Vector4d &v )
    {
      return v.head<3>();
    }
    
    template<typename T, typename std::enable_if<std::is_base_of<cv::Point2d, T>::value>::type* = nullptr>
    cv::Point2d fwd_vector( const Eigen::Vector4d &v )
    {
      return cv::Point2d(v.x(),v.y());
    }
    
    template <typename T>
    T get_direction_vector( const Eigen::Matrix4d &base, const Point2D &end_point )
    {
      Eigen::Vector4d beam_end = Eigen::Vector4d( end_point.x(), end_point.y(), 0, 1 );
      Eigen::Vector4d w_p_mp = base * beam_end;
      w_p_mp = w_p_mp / w_p_mp[3];
      w_p_mp.normalize();
      return fwd_vector<T>(w_p_mp);
    }
  
  };
  
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
    
    const size_t &global_idx() const;
    
    template<typename T>
    T get_direction_vector( const Eigen::Matrix4d &tf ) const
    {
      return transform_helpers::get_direction_vector<T>(tf, end_point);
    }
    
    template<typename T>
    T transform(const Eigen::Matrix4d &tf) const
    {
      return transform_helpers::transform_pnt<T>(tf, end_point);
    }
    
    static std::shared_ptr<Beam> make_beam( size_t idx, double range, double angle, Point2D end_point );
  
  private:
    bool valid_beam;
    bool is_visible_;
  };
  
}
#endif //PROJECT_BEAM_H
