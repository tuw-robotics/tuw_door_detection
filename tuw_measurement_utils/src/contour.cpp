//
// Created by felix on 29.10.18.
//

#include <tuw_measurement_utils/contour.h>
#include <iostream>
#include <set>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace tuw;

Contour::Contour() : length_( 0.0 )
{
  is_door_candidate_ = false;
  candidate_color_ = cv::Scalar( 0, 255, 0 );
  assigned_color_ = cv::Scalar( 255, 255, 255 );
  line_segments_ = std::vector<LineSegment2DDetector::LineSegment>( 0 );
  beams_ = std::vector<std::shared_ptr<Beam>>( 0 );
  children_ = std::vector<std::shared_ptr<Contour>>( 0 );
  child_candidates_ = std::vector<std::shared_ptr<Contour>>( 0 );
}

Contour::Contour( boost::uuids::uuid uuid ) : length_( 0.0 ), uuid_( uuid )
{
  is_door_candidate_ = false;
  candidate_color_ = cv::Scalar( 0, 255, 0 );
  assigned_color_ = cv::Scalar( 255, 255, 255 );
  line_segments_ = std::vector<LineSegment2DDetector::LineSegment>( 0 );
  beams_ = std::vector<std::shared_ptr<Beam>>( 0 );
  children_ = std::vector<std::shared_ptr<Contour>>( 0 );
  child_candidates_ = std::vector<std::shared_ptr<Contour>>( 0 );
}

void Contour::push_back( std::shared_ptr<Beam> beam )
{
  if ( !beams_.empty())
  {
    length_ += beams_.back()->end_point.distanceTo( beam->end_point );
  }
  beams_.push_back( beam );
}

double Contour::length()
{
  if ( beams_.size() <= 2 )
  {
    return 0;
  }
  return length_;
}

void Contour::detectCorners( const size_t KERNEL_SIZE )
{
  std::vector<double> sobel;
  sobel.resize( KERNEL_SIZE );
  
  for ( size_t i = 0; i < sobel.size(); ++i )
  {
    if ( i < KERNEL_SIZE / 2.0 )
    {
      sobel[i] = -1;
    } else
    {
      sobel[i] = 1;
    }
  }
  
  std::vector<double> ranges;
  ranges.resize( beams_.size() + KERNEL_SIZE );
  size_t i = 0;
  for ( ; i < KERNEL_SIZE / 2.0; ++i )
  {
    ranges[i] = (*begin())->range;
  }
  
  for ( std::vector<std::shared_ptr<Beam>>::const_iterator it_beam = begin();
        it_beam != (end() - 1); ++it_beam, ++i )
  {
    const auto beam = *it_beam;
    ranges[i] = beam->range;
  }
  
  for ( ; i < ranges.size(); ++i )
  {
    ranges[i] = (*(end() - 1))->range;
  }
  
}

void Contour::cvConvexityDefects( WorldScopedMaps &_map )
{
  std::vector<cv::Point2i> end_points_stl;
  std::vector<int> hull;
  std::vector<cv::Vec4i> defects;
  //std::vector<cv::Vec4i> defects;
  
  end_points_stl.resize( beams_.size());
  for ( auto i = 0; i < end_points_stl.size(); ++i )
  {
    const auto end_point = _map.w2m( beams_[i]->end_point );
    end_points_stl[i].x = end_point.x();
    end_points_stl[i].y = end_point.y();
  }
  
  cv::convexHull( end_points_stl, hull );
  cv::convexityDefects( end_points_stl, cv::Mat( hull ), defects );
  
  convexity_defects_.resize( defects.size());
  
  std::cout << std::endl << std::endl;
  for ( int i = 0; i < defects.size(); ++i )
  {
    std::cout << defects[i][2] << ", " << defects[i][3] << std::endl;
    convexity_defects_[i].reset( new CVDefect( defects[i][0], defects[i][1], defects[i][2], defects[i][3] ));
  }
}

void Contour::cvDetectCorners()
{
  corner_points_.clear();
  
  cv::Mat corners_;
  cv::cornerHarris( rendering_, corners_, 7, 3, 0.1 );
  
  double min, max;
  cv::minMaxLoc( corners_, &min, &max );
  
  //@ToDo: threshold
  double thresh = (max - min) * 0.75;
  
  size_t idx = 0;
  for ( size_t i = 0; i < corners_.rows; ++i )
  {
    for ( size_t j = 0; j < corners_.cols; ++j )
    {
      if ( corners_.at<float>( j, i ) > thresh )
      {
        const auto color = cv::Scalar( 255 );
        std::unique_ptr<Corner> corner;
        corner.reset( new Corner( cv::Point2d( i, j ), corners_.at<float>( i, j ), idx ));
        
        corner_points_.push_back( std::move( corner ));
        
        idx++;
        //cv::circle(corner_img, cv::Point2d(i, j), 3, color);
      }
    }
  }
  
  std::set<size_t> removal;
  //Non maximum suppression
  for ( int ii = 0; ii < corner_points_.size(); ++ii )
  {
    for ( int jj = 0; jj < corner_points_.size(); ++jj )
    {
      
      if ( ii == jj )
      {
        continue;
      }
      
      auto c0 = corner_points_[ii]->point;
      auto c1 = corner_points_[jj]->point;
      double d = cv::norm( c0 - c1 );
      
      //@ToDo: use meter not pixel!!! -> from world scoped map
      if ( d > 2 )
      {
        continue;
      }
      
      if ( corner_points_[ii]->response > corner_points_[jj]->response )
      {
        removal.insert( corner_points_[jj]->idx );
      }
    }
  }
  
  corner_points_.erase( std::remove_if( corner_points_.begin(), corner_points_.end(),
                                        [&removal]( const std::unique_ptr<Corner> &c )
                                        {
                                          if ( removal.count( c->idx ) > 0 )
                                          {
                                            return true;
                                          }
                                          return false;
                                        } ),
                        corner_points_.end());
  
}

const std::vector<std::unique_ptr<Contour::Corner>> &Contour::getCorners()
{
  return corner_points_;
}

void Contour::renderInternal( WorldScopedMaps &map )
{
  rendering_ = cv::Mat::zeros( map.height(), map.width(), CV_8U );
  cv::Scalar color_ = cv::Scalar( 255 );
  render( map, rendering_, color_, 2 );
}

void Contour::render( WorldScopedMaps &map, cv::Mat &img, cv::Scalar &color, double rad, bool corners )
{
  assigned_color_ = color;
  
  if ( is_door_candidate_ )
  {
    assigned_color_ = candidate_color_;
  }
  
  for ( std::vector<std::shared_ptr<Beam>>::const_iterator it_beam = begin();
        it_beam != (end() - 1); ++it_beam )
  {
    map.line( img, (*it_beam)->end_point, (*(it_beam + 1))->end_point, assigned_color_, rad );
  }
  
  if ( corners )
  {
    
    const auto ccolor = cv::Scalar( 0, 0, 255 );
    for ( const auto &c : corner_points_ )
    {
      cv::circle( img, c->point, 3, ccolor, 3 );
    }
    
    const auto cvdcolor = cv::Scalar( 0, 255, 0 );
    for ( const auto &d : convexity_defects_ )
    {
      map.line( img, beams_[d->start_idx]->end_point, beams_[d->end_idx]->end_point, cvdcolor, 1 );
    }
  }
}

void Contour::detectLines( LineSegment2DDetector &lineSegment2DDetector )
{
  
  //@ToDo: performance create views or whatever, avoid copying
  std::vector<Point2D> all_pnts = std::vector<Point2D>( beams_.size());
  
  //fill
  size_t each = 0;
  std::for_each( beams_.begin(), beams_.end(), [&each, &all_pnts]( const std::shared_ptr<Beam> &elem )
  {
    all_pnts[each++] = elem->end_point;
  } );
  
  lineSegment2DDetector.start( all_pnts );
  line_segments_ = lineSegment2DDetector.result();
}

bool Contour::optimizeLines( const unsigned int iterations )
{
  return true;
}

Point2D Contour::pointToImage( Eigen::Vector4d &pnt,
                               double fx, double fy,
                               double cx, double cy,
                               double tx, double ty )
{
  double x = (fx * pnt.x() + tx) / pnt.z() + cx;
  double y = (fy * pnt.y() + ty) / pnt.z() + cy;
  return Point2D( x, y );
}

void Contour::registerToImage( const Eigen::Matrix4d &tf, const double z_laser,
                               double fx, double fy,
                               double cx, double cy,
                               double tx, double ty )
{
  
  for ( std::shared_ptr<Beam> &elem : beams_ )
  {
    Eigen::Vector4d end_pnt( elem->end_point.x(), elem->end_point.y(), 0, 1 );
    end_pnt = tf * end_pnt;
    end_pnt = end_pnt / end_pnt[3];
    elem->img_coords = pointToImage( end_pnt, fx, fy, cx, cy, tx, ty );
    
    Eigen::Vector4d base( elem->end_point.x(), elem->end_point.y(), -z_laser, 1 );
    base = tf * base;
    base = base / base[3];
    elem->img_base_coords = pointToImage( base, fx, fy, cx, cy, tx, ty );
    
  }
  
  for ( LineSegment2D &ls : line_segments_ )
  {
    Eigen::Vector4d end_pnt( ls.x0(), ls.y0(), 0, 1 );
    {
      end_pnt = tf * end_pnt;
      end_pnt = end_pnt / end_pnt[3];
    }
    
    double x = (fx * end_pnt.x() + tx) / end_pnt.z() + cx;
    double y = (fy * end_pnt.y() + ty) / end_pnt.z() + cy;
    Point2D p0_img = Point2D( x, y );
    
    end_pnt = Eigen::Vector4d( ls.x1(), ls.y1(), 0, 1 );
    {
      end_pnt = tf * end_pnt;
      end_pnt = end_pnt / end_pnt[3];
    }
    
    x = (fx * end_pnt.x() + tx) / end_pnt.z() + cx;
    y = (fy * end_pnt.y() + ty) / end_pnt.z() + cy;
    Point2D p1_img = Point2D( x, y );
    
    line_segment_img_coords_.push_back( std::make_pair( p0_img, p1_img ));
  }
  
  //bottom left, top left, top right, bottom right
  std::vector<std::shared_ptr<Beam>> beams_ends;
  beams_ends.push_back( beams().front());
  beams_ends.push_back( beams().back());
  
  calculateBoundingBox( tf, z_laser, fx, fy, cx, cy, tx, ty );
  for ( auto &ch : children_ )
  {
    ch->calculateBoundingBox( tf, z_laser, fx, fy, cx, cy, tx, ty );
  }
}

void Contour::calculateBoundingBoxObjSpace()
{
  std::vector<std::shared_ptr<Beam>> beams_ends;
  beams_ends.push_back( beams().front());
  beams_ends.push_back( beams().back());
  
  for ( auto &beam : beams_ends )
  {
    bb_objspace_.push_back( Eigen::Vector3d( beam->end_point.x(), beam->end_point.y(), -0.05 ));
    bb_objspace_.push_back( Eigen::Vector3d( beam->end_point.x(), beam->end_point.y(), 0.05 ));
  }
  
  std::swap( bb_objspace_[2], bb_objspace_[3] );
}

void Contour::calculateBoundingBox( Eigen::Matrix4d tf, double z_laser,
                                    double fx, double fy,
                                    double cx, double cy,
                                    double tx, double ty )
{
  
  //bottom left, top left, top right, bottom right
  std::vector<std::shared_ptr<Beam>> beams_ends;
  beams_ends.push_back( beams().front());
  beams_ends.push_back( beams().back());
  
  bounding_box2laseridxs_.resize(2);
  bounding_box2laseridxs_[0] = std::min(beams().front()->global_idx_, beams().back()->global_idx_);
  bounding_box2laseridxs_[1] = std::max(beams().front()->global_idx_, beams().back()->global_idx_);
  
  bb_.clear();
  bb_objspace_.clear();
  
  //@ToDo: magic numbers
  size_t i_bm = 0;
  for ( auto &beam : beams_ends )
  {
    
    auto dir_vec = Eigen::Vector2d( beam->end_point.x(),
                                    beam->end_point.y());
    Eigen::Vector2d dn = dir_vec.normalized();
    
    bb_objspace_.push_back( Eigen::Vector3d( beam->end_point.x(), beam->end_point.y(), -z_laser ));
    bb_objspace_.push_back( Eigen::Vector3d( beam->end_point.x(), beam->end_point.y(), 2.0 - z_laser ));
    
    Eigen::Vector4d vbot = tf * Eigen::Vector4d( beam->end_point.x(),
                                                 beam->end_point.y(),
                                                 -(z_laser + 0.25),
                                                 1 );
    Eigen::Vector4d vtop = tf * Eigen::Vector4d( beam->end_point.x(),
                                                 beam->end_point.y(),
                                                 0.25 + 2.0 - z_laser,
                                                 1 );
    vbot = vbot / vbot[3];
    vtop = vtop / vtop[3];
    
    Point2D bot = pointToImage( vbot, fx, fy, cx, cy, tx, ty );
    Point2D top = pointToImage( vtop, fx, fy, cx, cy, tx, ty );
    bb_.push_back( bot );
    bb_.push_back( top );
    i_bm++;
    
  }
  
  std::swap( bb_[2], bb_[3] );
  std::swap( bb_objspace_[2], bb_objspace_[3] );
}

void Contour::visibilityCheck( bool shift_lines, int img_width, int img_height )
{
  unsigned int cnt = 0;
  for ( auto &b : beams_ )
  {
    bool not_vis = b->img_coords.x() < 0 || b->img_coords.x() > img_height || b->img_coords.y() < 0 ||
                   b->img_coords.y() > img_width;
    b->set_is_visible( !not_vis );
    if ( b->get_is_visible())
      cnt++;
  }
  std::cout << "visible beams: " << cnt << std::endl;
}
