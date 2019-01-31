//
// Created by felix on 30.01.19.
//

#include <datastructures/contour_container.h>

//TODO: remove
#include <iostream>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace tuw;

ContourContainer::ContourContainer( tuw::ContourContainer::Order order )
{
  sorting_order_ = order;
}

void ContourContainer::assertBeamOrder( const std::shared_ptr<tuw::Contour> &c )
{
  int idx;
  if ( sorting_order_ == Order::RIGHT2LEFT )
  {
    idx = std::numeric_limits<int>::min();
  } else
  {
    idx = std::numeric_limits<int>::max();
  }
  
  for ( const auto &b : c->beams())
  {
    if ( sorting_order_ == Order::RIGHT2LEFT )
    {
      assert( b->global_idx_ > idx );
      idx = b->global_idx_;
    } else if ( sorting_order_ == Order::LEFT2RIGHT )
    {
      assert( b->global_idx_ < idx );
      idx = b->global_idx_;
    }
  }
}

void ContourContainer::clear()
{
  id2contour_map_.clear();
  contours_sorted_.clear();
}

std::vector<std::shared_ptr<Contour>>::iterator ContourContainer::begin()
{
  contours_sorted_.begin();
}

std::vector<std::shared_ptr<Contour>>::iterator ContourContainer::end()
{
  contours_sorted_.end();
}

void ContourContainer::sortLines()
{
  for ( std::shared_ptr<Contour> contour : contours_sorted_ )
  {
    sortLines( contour );
  }
}

void ContourContainer::sortLines( std::shared_ptr<Contour> &c )
{
  std::cout << "sort" << std::endl;
  std::vector<LineSegment2DDetector::LineSegment> &line_segments = c->getLineSegments();
  std::sort( line_segments.begin(), line_segments.end(),
             [this]( const LineSegment2DDetector::LineSegment &c0, const LineSegment2DDetector::LineSegment &c1 )
             {
               if ( sorting_order_ == Order::RIGHT2LEFT )
               {
                 return c0.idx0_ < c1.idx0_;
               } else if ( sorting_order_ == Order::LEFT2RIGHT )
               {
                 return c0.idx0_ > c1.idx0_;
               }
             } );
  std::cout << "sort" << std::endl;
}

size_t ContourContainer::size()
{
  return contours_sorted_.size();
}

std::shared_ptr<Contour> ContourContainer::getNeighborNext( const std::shared_ptr<Contour> &c )
{
  auto it = id2contour_map_.find( c->id());
  if ( it != id2contour_map_.end())
  {
    unsigned int idx = it->second;
    if ( idx >= 0 && idx < (contours_sorted_.size() - 1))
    {
      return contours_sorted_[idx + 1];
    }
  }
  return nullptr;
}

std::shared_ptr<Contour> ContourContainer::getNeighborPrev( const std::shared_ptr<tuw::Contour> &c )
{
  auto it = id2contour_map_.find( c->id());
  if ( it != id2contour_map_.end())
  {
    unsigned int idx = it->second;
    if ( idx > 0 && idx <= (contours_sorted_.size() - 1))
    {
      return contours_sorted_[idx - 1];
    }
  }
  return nullptr;
}

void ContourContainer::insert( const std::vector<std::shared_ptr<Contour>> &contours_unsorted )
{
  for ( const std::shared_ptr<Contour> &c : contours_unsorted )
  {
    assertBeamOrder( c );
  }
  
  contours_sorted_.resize( contours_unsorted.size());
  std::copy( contours_unsorted.begin(), contours_unsorted.end(), contours_sorted_.begin());
  
  if ( sorting_order_ == Order::RIGHT2LEFT )
  {
    std::sort( contours_sorted_.begin(), contours_sorted_.end(),
               []( const std::shared_ptr<Contour> &c0, const std::shared_ptr<Contour> &c1 )
               {
                 return c0->beams().front()->global_idx_ < c1->beams().front()->global_idx_;
               } );
  } else
  {
    std::sort( contours_sorted_.begin(), contours_sorted_.end(),
               []( const std::shared_ptr<Contour> &c0, const std::shared_ptr<Contour> &c1 )
               {
                 return c0->beams().front()->global_idx_ > c1->beams().front()->global_idx_;
               } );
  }
  
  assertContourOrder();
  
  if ( contours_sorted_.size() > 1 )
  {
    std::cout << "uuid front " << boost::lexical_cast<std::string>( contours_sorted_.front()->id()) << std::endl;
    std::cout << "uuid back " << boost::lexical_cast<std::string>( contours_sorted_.back()->id()) << std::endl;
  }
  
  unsigned int ii = 0;
  for ( std::shared_ptr<Contour> c : contours_sorted_ )
  {
    id2contour_map_.insert( std::make_pair( c->id(), ii++ ));
  }
}

std::vector<std::shared_ptr<Contour>> &ContourContainer::getContours()
{
  return contours_sorted_;
}

void ContourContainer::assertContourOrder()
{
  if ( sorting_order_ == Order::RIGHT2LEFT )
  {
    double constraint_right2left = contours_sorted_.front()->beams().front()->global_idx_;
    std::for_each( contours_sorted_.begin() + 1, contours_sorted_.end(),
                   [&constraint_right2left]( std::shared_ptr<Contour> c )
                   {
                     double idx = c->front()->global_idx_;
                     assert( idx > constraint_right2left );
                     constraint_right2left = idx;
                   } );
  } else if ( sorting_order_ == Order::LEFT2RIGHT )
  {
    double constraint_left2right = contours_sorted_.front()->beams().front()->global_idx_;
    std::for_each( contours_sorted_.begin() + 1, contours_sorted_.end(),
                   [&constraint_left2right]( std::shared_ptr<Contour> c )
                   {
                     double idx = c->front()->global_idx_;
                     assert( idx < constraint_right2left );
                     constraint_left2right = idx;
                   } );
  }
}

