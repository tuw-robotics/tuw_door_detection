//
// @author felix.koenig@protonmail.com
//
// Created by felix on 30.01.19.
//

#ifndef TUW_DOOR_DETECTION_DATASTRUCTURES_CONTOURTREE_H
#define TUW_DOOR_DETECTION_DATASTRUCTURES_CONTOURTREE_H

#include <boost/circular_buffer.hpp>
#include <boost/uuid/uuid.hpp>
#include <vector>
#include <map>
#include <tuw_measurement_utils/contour.h>

namespace tuw
{
  class ContourContainer
  {
  public:
    enum class Order
    {
      LEFT2RIGHT = 1000,
      RIGHT2LEFT
    };
  
  public:
    
    ContourContainer( Order order = Order::RIGHT2LEFT );
    
    void assertBeamOrder( const std::shared_ptr<Contour> &c );
    
    void assertContourOrder();
    
    void clear();
    
    std::vector<std::shared_ptr<Contour>>::iterator begin();
    
    std::vector<std::shared_ptr<Contour>>::iterator end();
    
    size_t size();
    
    std::vector<std::shared_ptr<Contour>> &getContours();
    
    std::shared_ptr<Contour> getNeighborNext( const std::shared_ptr<Contour> &c );
    
    std::shared_ptr<Contour> getNeighborPrev( const std::shared_ptr<Contour> &c );
    
    std::shared_ptr<Contour> getEuclideanClosest( const std::shared_ptr<Contour> &c );
    
    void sortLines();
    
    void sortLines( std::shared_ptr<Contour> &c );
    
    /**
     * Inserts the contours into the vector sorted either from right to left or left to right (as seen from laser scan)
     *
     * @param contours_unsorted: The beams must be sorted in the correct order -> otherwise fails
     */
    void insert( const std::vector<std::shared_ptr<Contour>> &contours_unsorted );
  
  private:
    Order sorting_order_;
    std::vector<std::shared_ptr<Contour>> contours_sorted_;
    std::map<boost::uuids::uuid, unsigned int> id2contour_map_;
  };
}

#endif //PROJECT_CONTOURTREE_H
