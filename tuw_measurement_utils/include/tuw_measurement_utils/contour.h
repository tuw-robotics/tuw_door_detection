//
// Created by felix on 29.10.18.
//

#ifndef TUW_MEASUREMENT_UTILS_CONTOUR_H
#define TUW_MEASUREMENT_UTILS_CONTOUR_H

#include <tuw_geometry/point2d.h>
#include <memory>
#include <tuw_geometry/world_scoped_maps.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <Eigen/Core>
#include <boost/uuid/uuid.hpp>
#include <tuw_measurement_utils/beam.h>

namespace tuw
{
  
  class Contour
  {
  
  public:
    
    //class CandidateAttributes
    //{
    //public:
    //  void setLengthRequirement()
    //  {
    //    length_requirement_ = true;
    //  }
    //
    //  void setCrossingSegs()
    //  {
    //    crossing_segs_ = true;
    //  }
    //
    //  void setDoor
    //
    //private:
    //  bool length_requirement_;
    //  bool crossing_segs_;
    //  bool door_candidate_;
    //};
    
    
    
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
    
    Contour( boost::uuids::uuid uuid );
    
    Contour();
    
    boost::uuids::uuid id()
    {
      return uuid_;
    }
    
    const boost::uuids::uuid id() const;
    
    void push_back( std::shared_ptr<Beam> beam );
    
    void detectCorners( const size_t KERNEL_SIZE );
    
    void doorPostLeft( const bool left_post )
    {
      door_post_left_ = left_post;
    }
    
    const bool doorPostLeft() const
    {
      return door_post_left_;
    }
    
    void cvConvexityDefects( tuw::WorldScopedMaps &_map );
    
    void cvDetectCorners();
    
    void detectLines( LineSegment2DDetector &lineSegment2DDetector );
    
    Point2D pointToImage( Eigen::Vector4d &pnt,
                          double fx, double fy,
                          double cx, double cy,
                          double tx, double ty );
    
    void registerToImage( const Eigen::Matrix4d &tf, const double z_laser,
                          double fx, double fy,
                          double cx, double cy,
                          double tx, double ty );
    
    void visibilityCheck( bool shift_lines, int img_width, int img_height );
    
    const std::vector<std::unique_ptr<Corner>> &getCorners();
    
    const std::vector<LineSegment2DDetector::LineSegment> &getLineSegments() const
    {
      return line_segments_;
    };
    
    std::vector<LineSegment2DDetector::LineSegment> &getLineSegments()
    {
      return line_segments_;
    }
    
    const std::vector<std::pair<Point2D, Point2D>> &getLineSegmentImageCoords()
    {
      return line_segment_img_coords_;
    }
    
    void renderInternal( tuw::WorldScopedMaps &map );
    
    bool is_door_candidate()
    {
      return is_door_candidate_;
    }
    
    void set_door_candidate( bool val )
    {
      this->is_door_candidate_ = val;
    }
    
    void candidateLikelyhood( const double lh )
    {
      likelyhood_ = lh;
    }
    
    const double candidateLikelyhood() const
    {
      return likelyhood_;
    }
    
    std::shared_ptr<Beam> front()
    {
      beams_.front();
    }
    
    std::shared_ptr<Beam> back()
    {
      beams_.back();
    }
    
    std::vector<std::shared_ptr<Beam>> &beams()
    {
      return beams_;
    }
    
    const std::vector<std::shared_ptr<Beam>> &beams() const
    {
      return beams_;
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
    
    const std::vector<std::shared_ptr<Contour>> &getChildren() const
    {
      return children_;
    }
    
    std::vector<std::shared_ptr<Contour>> &getChildren()
    {
      return children_;
    }
    
    void addChild( std::shared_ptr<Contour> &child )
    {
      children_.push_back( child );
    }
    
    std::vector<std::shared_ptr<Contour>> &getChildCandidates()
    {
      return child_candidates_;
    }
    
    void addChildCandidate( std::shared_ptr<Contour> &child_candidate )
    {
      child_candidates_.push_back( child_candidate );
    }
    
    const cv::Scalar &getAssignedColor() const
    {
      return assigned_color_;
    }
    
    void setBoundingBox( std::vector<Point2D> &bb )
    {
      bb_ = bb;
    }
    
    const std::vector<Eigen::Vector3d> &getBoundingBoxObjSpace() const
    {
      return bb_objspace_;
    }
    
    std::vector<Eigen::Vector3d> &getBoundingBoxObjSpace()
    {
      return bb_objspace_;
    }
    
    std::vector<cv::Point2d> getBoundingBoxCV()
    {
      return std::vector<cv::Point2d>{
          bb_[0].cv(),
          bb_[0].cv(),
          bb_[0].cv(),
          bb_[0].cv()
      };
    }
    
    std::vector<Point2D> &getBoundingBox()
    {
      return bb_;
    }
    
    void calculateBoundingBoxObjSpace();
    
    void calculateBoundingBox( Eigen::Matrix4d tf, double z_laser,
                               double fx, double fy,
                               double cx, double cy,
                               double tx, double ty );
    
    double length();
    
    bool optimizeLines( const unsigned int iterations );
  
  private:
    std::vector<std::shared_ptr<Beam>> beams_;
    std::vector<std::unique_ptr<Corner>> corner_points_;
    std::vector<LineSegment2DDetector::LineSegment> line_segments_;
    std::vector<std::pair<Point2D, Point2D>> line_segment_img_coords_;
    std::vector<std::shared_ptr<Contour>> children_;
    std::vector<std::shared_ptr<Contour>> child_candidates_;
    bool door_post_left_;
    boost::uuids::uuid uuid_;
    bool length_cache_uptodate_;
    double length_;
    double likelyhood_;
    size_t num_corners_;
    cv::Mat rendering_;
    bool is_door_candidate_;
    
    cv::Scalar assigned_color_;
    cv::Scalar candidate_color_;
    
    std::vector<std::unique_ptr<CVDefect>> convexity_defects_;
    std::vector<Point2D> bb_;
    std::vector<Eigen::Vector3d> bb_objspace_;
  };
};


#endif //PROJECT_CONTOUR_H
