#ifndef DOOR_DEPTH_DETECTOR_H
#define DOOR_DEPTH_DETECTOR_H

#include "door_detector.h"
#include "door_detection.h"
#include <ros/publisher.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tuw_geometry/figure.h>
#include <tuw_door_detection/DepthDetectorConfig.h>
#include <dynamic_reconfigure/server.h>

namespace tuw {
  class DoorDepthDetector : public DoorDetector {
  
  public:
    struct ParametersNode {
      ParametersNode();
      
      ros::NodeHandle node;
      bool debug;
      std::string world_frame;
      std::string source_frame;
      std::string publisher_topic;
      std::string internal_mode;
    };
    
    //TODO: use callback from parameterserver
    struct Config {
      Config() {
        map_pix_x = 500;
        map_pix_y = 500;
        map_max_x = 5;
        map_max_y = 5;
        map_min_x = -5;
        map_min_y = -5;
        map_grid_x = 1;
        map_grid_y = 1;
        map_rotation = 0;
      }
      
      double map_pix_x;
      double map_pix_y;
      double map_max_x;
      double map_max_y;
      double map_min_x;
      double map_min_y;
      double map_grid_x;
      double map_grid_y;
      double map_rotation;
    };
    
    std::unique_ptr<ParametersNode> params_;
    std::unique_ptr<Config> config_;
    
    ///// parameter server for dynamic detector configuration
    // dynamic_reconfigure::Server<tuw_door_detection::DepthDetectorConfig> reconfigure_server_;
    
    // /// parameter server callback
    // dynamic_reconfigure::Server<tuw_door_detection::DepthDetectorConfig>::CallbackType reconfigure_fnc_;
    
    ///**
    //  * @brief callback function on incoming parameter changes
    //  * @param config the configuration message
    //  * @param level not used here, but required for dynamic reconfigure callbacks
    //  */
    // void callbackConfig(tuw_door_detection::DepthDetectorConfig &_config, uint32_t level);
    
    DoorDepthDetector( ros::NodeHandle &_nh );
    
    virtual ~DoorDepthDetector();
    
    WorldScopedMaps &getMap() { return ws_map_; }
  
  protected:
    //std::unique_ptr<tuw_door_detection::DepthDetectorConfig> config_;
    
    bool processLaser( const sensor_msgs::LaserScan &_laser ) override;
    
    void plot( const std::vector<double> &_responses );
    
    void plot( const std::vector<DoorDetectionPtr> &_detections );
    
    template<typename T>
    std::vector<T> normalize( std::vector<T> &_resp );
    
    WorldScopedMaps ws_map_;
  
  private:
    Figure figure_local_;
    float thresh_{0.3};
    std::size_t KERNEL_SIZE = {8};
    ros::Publisher pubObjectDetections_;
    std::map<size_t, cv::Scalar> colorMap_;
    
    std::unique_ptr<ParametersNode> &params() { return params_; }
    
    bool structureMode( const sensor_msgs::LaserScan &_laser, std::vector<DoorDetectionPtr> &_detections );
    
    bool kernelMode( const sensor_msgs::LaserScan &_laser, std::vector<DoorDetectionPtr> &_detections );
    
    bool contourMode( const sensor_msgs::LaserScan &_laser );
  };
}

#endif // DOOR_DEPTH_DETECTOR_H
