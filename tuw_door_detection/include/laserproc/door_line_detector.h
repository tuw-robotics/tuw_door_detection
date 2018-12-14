#ifndef DOOR_LINE_DETECTOR_H
#define DOOR_LINE_DETECTOR_H

#include "door_detector_base.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
//#include <tuw_geometry/measurement_laser.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <dynamic_reconfigure/server.h>

namespace tuw {

  namespace door_laser_proc {

    class DoorLineDetector : public LineSegment2DDetector, public DoorDetectorBase {
    public:
      DoorLineDetector(ros::NodeHandle &_nh);

      virtual ~DoorLineDetector();

    protected:
      /**
       * @brief callback function for incoming laser scans
       * @param _laser laser scan message
       */
      bool processLaser(const sensor_msgs::LaserScan &_laser) override;

    private:
      enum FilterMode {
        FILTER_DOORS, FILTER_NON_DOORS
      };
      //MeasurementLaserPtr measurement_laser_;                /// laser measurements
      std::vector<Point2D> measurement_local_scanpoints_;    /// laser beam endpoints for line detection
      std::vector<LineSegment> measurement_linesegments_;  /// detected line segments in sensor coordinates
      ros::Publisher line_pub_;
      ros::Publisher door_pub_;
      ros::Publisher laser_pub_;
      bool display_window_;
      bool modify_laser_scan_;
      FilterMode doors_filter_mode_;
      std::pair<double, double> door_range;

      //   /// parameter server for dynamic detector configuration
      //   dynamic_reconfigure::Server<tuw_geometry::Linesegment2DDetectorConfig> reconfigure_server_;
      //
      //   /// parameter server callback
      //   dynamic_reconfigure::Server<tuw_geometry::Linesegment2DDetectorConfig>::CallbackType reconfigure_fnc_;

      /**
     * @brief callback function on incoming parameter changes
     * @param config the configuration message
     * @param level not used here, but required for dynamic reconfigure callbacks
     */
      //void callbackConfig( tuw_geometry::Linesegment2DDetectorConfig &config, uint32_t level );


      bool is_in_doorrange(tuw_geometry_msgs::LineSegment &line_segment);

    };

  };
 
}

#endif // DOOR_LINE_DETECTOR_H
