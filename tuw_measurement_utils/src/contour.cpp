//
// Created by felix on 29.10.18.
//

#include <tuw_measurement_utils/contour.h>
#include <iostream>
#include <set>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

tuw::Contour::Contour() : length_( 0.0 ) {
  
}

tuw::Contour::Beam::Beam( double _range, double _angle, tuw::Point2D _end_point ) {
  range = _range;
  angle = _angle;
  end_point = _end_point;
  valid_beam = true;
}

const bool tuw::Contour::Beam::is_valid() const {
  return valid_beam;
}

void tuw::Contour::Beam::set_valid( const bool v ) {
  valid_beam = v;
}

std::shared_ptr<tuw::Contour::Beam>
tuw::Contour::Beam::make_beam( double range, double angle, tuw::Point2D end_point ) {
  return std::make_shared<Beam>( range, angle, end_point );
}

void tuw::Contour::push_back( std::shared_ptr<tuw::Contour::Beam> beam ) {
  if ( beams_.size()) {
    length_ += beams_.back()->end_point.distanceTo( beam->end_point );
  }
  beams_.push_back( beam );
}

double tuw::Contour::length() {
  if ( beams_.size() <= 2 ) {
    return 0;
  }
  return length_;
}

void tuw::Contour::detectCorners( const size_t KERNEL_SIZE ) {
  std::vector<double> sobel;
  sobel.resize( KERNEL_SIZE );
  
  for ( size_t i = 0; i < sobel.size(); ++i ) {
    if ( i < KERNEL_SIZE / 2.0 ) {
      sobel[i] = -1;
    } else {
      sobel[i] = 1;
    }
  }
  
  std::vector<double> ranges;
  ranges.resize( beams_.size() + KERNEL_SIZE );
  size_t i = 0;
  for ( ; i < KERNEL_SIZE / 2.0; ++i ) {
    ranges[i] = (*begin())->range;
  }
  
  for ( std::vector<std::shared_ptr<Contour::Beam>>::const_iterator it_beam = begin();
        it_beam != (end() - 1); ++it_beam, ++i ) {
    const auto beam = *it_beam;
    ranges[i] = beam->range;
  }
  
  for ( ; i < ranges.size(); ++i ) {
    ranges[i] = (*(end() - 1))->range;
  }
  
}

void tuw::Contour::cvConvexityDefects( tuw::WorldScopedMaps &_map ) {
  std::vector<cv::Point2i> end_points_stl;
  std::vector<int> hull;
  std::vector<cv::Vec4i> defects;
  //std::vector<cv::Vec4i> defects;
  
  end_points_stl.resize( beams_.size());
  for ( auto i = 0; i < end_points_stl.size(); ++i ) {
    const auto end_point = _map.w2m( beams_[i]->end_point );
    end_points_stl[i].x = end_point.x();
    end_points_stl[i].y = end_point.y();
  }
  
  cv::convexHull( end_points_stl, hull );
  cv::convexityDefects( end_points_stl, cv::Mat( hull ), defects );
  
  convexity_defects_.resize( defects.size());
  
  std::cout << std::endl << std::endl;
  for ( int i = 0; i < defects.size(); ++i ) {
    std::cout << defects[i][2] << ", " << defects[i][3] << std::endl;
    convexity_defects_[i].reset( new CVDefect( defects[i][0], defects[i][1], defects[i][2], defects[i][3] ));
  }
}

void tuw::Contour::cvDetectCorners() {
  corner_points_.clear();
  
  cv::Mat corners_;
  cv::cornerHarris( rendering_, corners_, 7, 3, 0.1 );
  
  double min, max;
  cv::minMaxLoc( corners_, &min, &max );
  double thresh = max * 0.75;
  
  size_t idx = 0;
  for ( size_t i = 0; i < corners_.rows; ++i ) {
    for ( size_t j = 0; j < corners_.cols; ++j ) {
      if ( corners_.at<float>( j, i ) > thresh ) {
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
  for ( int ii = 0; ii < corner_points_.size(); ++ii ) {
    for ( int jj = 0; jj < corner_points_.size(); ++jj ) {
      
      if ( ii == jj ) {
        continue;
      }
      
      auto c0 = corner_points_[ii]->point;
      auto c1 = corner_points_[jj]->point;
      double d = cv::norm( c0 - c1 );
      
      if ( d > 5 ) {
        continue;
      }
      
      if ( corner_points_[ii]->response > corner_points_[jj]->response ) {
        removal.insert( corner_points_[jj]->idx );
      }
    }
  }
  
  corner_points_.erase( std::remove_if( corner_points_.begin(), corner_points_.end(),
                                        [&removal]( const std::unique_ptr<Corner> &c ) {
                                          if ( removal.count( c->idx ) > 0 ) {
                                            return true;
                                          }
                                          return false;
                                        } ),
                        corner_points_.end());
}

const std::vector<std::unique_ptr<tuw::Contour::Corner>> &tuw::Contour::getCorners() {
  return corner_points_;
}

void tuw::Contour::renderInternal( tuw::WorldScopedMaps &map ) {
  rendering_ = cv::Mat::zeros( map.height(), map.width(), CV_8U );
  cv::Scalar color_ = cv::Scalar( 255 );
  render( map, rendering_, color_, 2 );
}

void tuw::Contour::render( tuw::WorldScopedMaps &map, cv::Mat &img, cv::Scalar &color, double rad, bool corners ) {
  for ( std::vector<std::shared_ptr<Contour::Beam>>::const_iterator it_beam = begin();
        it_beam != (end() - 1); ++it_beam ) {
    map.line( img, (*it_beam)->end_point, (*(it_beam + 1))->end_point, color, rad );
  }
  
  if ( corners ) {
    const auto ccolor = cv::Scalar( 0, 0, 255 );
    for ( const auto &c : corner_points_ ) {
      cv::circle( img, c->point, 3, ccolor, 3 );
    }
    
    const auto cvdcolor = cv::Scalar( 0, 255, 0 );
    for ( const auto &d : convexity_defects_ ) {
      map.line( img, beams_[d->start_idx]->end_point, beams_[d->end_idx]->end_point, cvdcolor, 1 );
    }
  }
}
