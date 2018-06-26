#ifndef STATIC_DOOR_PUBLISHER_NODE
#define STATIC_DOOR_PUBLISHER_NODE

namespace tuw
{
/**
 * @brief ROS wrapper node for LineSegment2DDetector
 * @class Door2DDetectorNode
 */
  class StaticDoorPublisherNode
  {
  public:
    StaticDoorPublisherNode();

  private:
    enum FilterMode { FILTER_DOORS, FILTER_NON_DOORS };

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_laser_;  /// Subscriber to the laser measurements

    std::pair<double,double> door_range;

    /**
     * @brief callback function for incoming laser scans
     * @param _laser laser scan message
     */
    void callbackLaser(const sensor_msgs::LaserScan &_laser);

    bool is_in_doorrange(tuw_geometry_msgs::LineSegment &line_segment);
  };
};

#endif
