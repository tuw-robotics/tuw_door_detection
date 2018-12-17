//
// Created by felix on 29.10.18.
//

#ifndef CONTOUR_H
#define CONTOUR_H

#include "../../../../../tuw_ros_commons/src/tuw_geometry/include/tuw_geometry/point2d.h"
#include <memory>
#include "../../../../../tuw_ros_commons/src/tuw_geometry/include/tuw_geometry/world_scoped_maps.h"

namespace tuw
{
  
  class Contour
  {
  
  public:
    
    class Beam
    {
    public:
      Beam()
      {
      };
      
      Beam( double range, double angle, Point2D end_point );
      
      //Beam(const Beam &) = delete;
      
      //Beam &operator=(const Beam &) = delete;
      
      ~Beam() = default;
      
      Point2D end_point;
      double range;
      double angle;
      
      const bool is_valid() const;
      
      void set_valid( const bool v );
      
      static std::shared_ptr<Beam> make_beam( double range, double angle, Point2D end_point );
    
    private:
      bool valid_beam;
    };
    
    struct CVDefect
    {
      CVDefect( int startIdx, int endIdx, int x, int y )
      {
        start_idx = startIdx;
        end_idx = endIdx;
        point = cv::Vec2i( x, y );
      }
      
      cv::Vec2i point;
      int start_idx;
      int end_idx;
    };
    
    struct Corner
    {
      
      Corner( const cv::Point2d &_point, size_t _response, size_t _idx )
      {
        point = _point;
        response = _response;
        idx = _idx;
      }
      
      cv::Point2d point;
      double response;
      size_t idx;
    };
    
    Contour();
    
    void push_back( std::shared_ptr<Beam> beam );
    
    void detectCorners( const size_t KERNEL_SIZE );
    
    void cvConvexityDefects( tuw::WorldScopedMaps &_map );
    
    void cvDetectCorners();
    
    const std::vector<std::unique_ptr<Corner>> &getCorners();
    
    void renderInternal( tuw::WorldScopedMaps &map );
    
    bool is_door_candidate()
    {
      return is_door_candidate_;
    }
    
    void set_door_candidate( bool val )
    {
      this->is_door_candidate_ = val;
    }
    
    void render( WorldScopedMaps &map2image, cv::Mat &image, cv::Scalar &color, double rad = 2, bool corners = true );
    
    const Point2D &startPoint() const
    {
      const auto elem = *beams_.begin();
      assert ( elem );
      return elem->end_point;
    }
    
    const Point2D &endPoint() const
    {
      const auto elem = *beams_.end();
      assert ( elem );
      return elem->end_point;
    }
    
    Point2D &startPoint()
    {
      const auto elem = *beams_.begin();
      assert ( elem );
      return elem->end_point;
    }
    
    Point2D &endPoint()
    {
      const auto elem = *beams_.end();
      assert ( elem );
      return elem->end_point;
    }
    
    std::vector<std::shared_ptr<Beam>>::const_iterator begin() const
    {
      return beams_.begin();
    }
    
    std::vector<std::shared_ptr<Beam>>::iterator begin()
    {
      return beams_.begin();
    }
    
    std::vector<std::shared_ptr<Beam>>::const_iterator end() const
    {
      return beams_.end();
    }
    
    std::vector<std::shared_ptr<Beam>>::iterator end()
    {
      return beams_.end();
    }
    
    const cv::Scalar &getAssignedColor() const
    {
      return assigned_color_;
    }
    
    double length();
  
  private:
    std::vector<std::shared_ptr<Beam>> beams_;
    std::vector<std::unique_ptr<Corner>> corner_points_;
    bool length_cache_uptodate_;
    double length_;
    size_t num_corners_;
    cv::Mat rendering_;
    bool is_door_candidate_;
    
    cv::Scalar assigned_color_;
    cv::Scalar candidate_color_;
    
    std::vector<std::unique_ptr<CVDefect>> convexity_defects_;
  };
};


#endif //PROJECT_CONTOUR_H
