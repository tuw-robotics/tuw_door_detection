//
// Created by felix on 30.01.19.
//

#include <datastructures/contour_tree.h>

using namespace tuw;

ContourTree::ContourTree( tuw::ContourTree::Order order )
{
  sorting_order_ = order;
}

void ContourTree::assertBeamOrder( const std::shared_ptr<tuw::Contour> &c )
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

void ContourTree::insert( const std::vector<std::shared_ptr<Contour>> &contours_unsorted )
{
  for ( const std::shared_ptr<Contour> &c : contours_unsorted )
  {
    assertBeamOrder( c );
  }
  
  contours_sorted_ = contours_unsorted;
  
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
  
  for ( std::shared_ptr<Contour> c : contours_sorted_ )
  {
    id2contour_map_.insert( std::make_pair( c->id(), c ));
  }
}

void ContourTree::assertContourOrder()
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

