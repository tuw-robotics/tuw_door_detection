#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>
#include "contour.h"
#include <image_geometry/pinhole_camera_model.h>

namespace tuw {
  
  class Measurement {
  public:
    
    Measurement( const Eigen::Matrix<double, 4, 4> &tfWorldSensor );
    
    Measurement( const tf::StampedTransform &_tf );
    
    ~Measurement() {
    }
    
    const Eigen::Matrix<double, 4, 4> &getTfWorldSensor() const;
    
    void setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor );
    
    void setTfWorldSensor( const tf::StampedTransform &_tf );
    
    void vectorQuaternionToEigen( const tf::Point &pt, const tf::Quaternion &q, Eigen::Matrix<double, 4, 4> &m );
    
    void cv2Eigen( const cv::Matx44d &_we_dont_want, Eigen::Matrix<double, 4, 4> &_we_want );
  
  protected:
    Eigen::Matrix<double, 4, 4> tfWorldSensor;
  };
  
  class LaserMeasurement : public Measurement {
  protected:
    sensor_msgs::LaserScan laser;
    std::vector<Contour::Beam> beams_;
  
  public:
    LaserMeasurement( const sensor_msgs::LaserScan &_laser, const tf::StampedTransform &_tf );
    
    ~LaserMeasurement() {
    }
    
    void initFromScan();
    
    const sensor_msgs::LaserScan &getLaser() const;
    
    void push_back( const Contour::Beam &_beam );
    
    void resize( const size_t _sz );
    
    const size_t size() const {
      return beams_.size();
    }
    
    size_t size() {
      return beams_.size();
    }
    
    std::vector<Contour::Beam>::iterator begin();
    
    std::vector<Contour::Beam>::iterator end();
    
    const Contour::Beam &operator[]( const size_t _sz ) const;
    
    Contour::Beam &operator[]( const size_t _sz );
  };
  
  class ImageMeasurement : public Measurement {
  
  public:
    
    ImageMeasurement( const cv_bridge::CvImageConstPtr &, const tf::StampedTransform & );
    
    ImageMeasurement( const tf::StampedTransform &_tf, const cv_bridge::CvImagePtr &image );
    
    ImageMeasurement( const cv_bridge::CvImagePtr &, const tf::StampedTransform &,
                      const sensor_msgs::CameraInfo & );
    
    ImageMeasurement( const cv_bridge::CvImagePtr &_image, const tf::StampedTransform &_tf );
    
    ~ImageMeasurement() {
    }
    
    cv_bridge::CvImagePtr &getImage();
    
    cv::Mat &getCVImage();
    
    const std::shared_ptr<image_geometry::PinholeCameraModel> &getCameraModel() const;
    
    std::shared_ptr<image_geometry::PinholeCameraModel> &getCameraModel();
    
    void setImage( const cv_bridge::CvImagePtr &image );
    
    void setCameraModel( const std::shared_ptr<image_geometry::PinholeCameraModel> &camera_model_ptr );
  
  protected:
    cv_bridge::CvImagePtr image;
    std::shared_ptr<image_geometry::PinholeCameraModel> camera_;
  };
};

#endif
