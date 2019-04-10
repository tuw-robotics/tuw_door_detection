//
// Created by felix on 14.12.18.
//

#ifndef TUW_DOOR_DETECTOR_H
#define TUW_DOOR_DETECTOR_H

#include <imgproc/door_detector_imgproc.h>
#include <laserproc/door_depth_detector.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <tuw_measurement_utils/laser_measurement.h>
#include <datastructures/octo_object_map.h>

namespace tuw
{
  
  class DoorDetector
  {
  public:
    
    
    DoorDetector();
    
    ~DoorDetector();
    
    bool merge( std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
                std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor );
    
    void setImageMeasurement( std::shared_ptr<ImageMeasurement> &image_meas );
    
    void setLaserMeasurement( std::shared_ptr<LaserMeasurement> &laser_meas );
    
    /**
     * setting the transform from world into baselink coordinates
     * it is used for a more reliable redetection of doors
     * */
    void setRobotPosition( Eigen::Matrix4d &tf_world_baselink );
    
    /**
     * Uses a kd tree for lookup of poses.
     */
    bool lookupHistory( const std::shared_ptr<Contour> &contr, Eigen::Matrix4d &tf, bool addifnotfound = false );
    
    void clear();
    
    void display();
    
    std::vector<std::shared_ptr<Contour>> &getDoorsLaser()
    {
      return doors_;
    }
    
    tuw_object_msgs::ObjectWithCovariance generateObjMessage( std::shared_ptr<Contour> &_contour, int32_t id );
    
    tuw_object_msgs::ObjectDetection getResultAsMessage();
    
    tuw_object_msgs::ObjectDetection getMappedDoorsAsMessage();
    
    void draw_roi( std::shared_ptr<Contour> &contour, cv::Mat &img_display );
  
  private:
    
    //void update(Eigen::Matrix4d &tf);
    
    void printOctree();
    
    void addOctNode( std::shared_ptr<Contour> &contour, Eigen::Matrix4d &tf );
    
    std::shared_ptr<OctoObjectMap> octo_object_map_;
    std::shared_ptr<Eigen::Matrix4d> tf_world_baselink_;
    std::shared_ptr<ImageMeasurement> image_measurement_;
    std::shared_ptr<LaserMeasurement> laser_measurement_;
    image_processor::DoorDetectionPtr detection_image_;
    std::vector<std::shared_ptr<Contour>> detection_laser_;
    std::vector<std::shared_ptr<Contour>> doors_;
    std::vector<std::shared_ptr<Contour>> door_candidates_;
    double door_height_ = 2.0;
    double door_width_ = 0.9;
    double bb_h_off_ = 0.15;
  };
  
};

#endif //PROJECT_DOOR_DETECTOR_H
