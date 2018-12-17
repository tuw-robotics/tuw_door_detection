#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include "contour.h"
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/TransformStamped.h>

namespace tuw
{
  
  class Measurement
  {
  public:
    
    Measurement( const Eigen::Matrix<double, 4, 4> &tfWorldSensor );
    
    Measurement( const geometry_msgs::TransformStampedPtr _tf );
    
    ~Measurement()
    {
    }
    
    const Eigen::Matrix<double, 4, 4> &getTfWorldSensor() const;
    
    void setTfWorldSensor( cv::Mat &_tf );
    
    void setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor, bool override_stamped = false );
    
    void setTfWorldSensor( const geometry_msgs::TransformStampedPtr _tf );
    
    const geometry_msgs::TransformStampedPtr getStampedTf() const;
    
    void vectorQuaternionToEigen( const geometry_msgs::Transform &tf, Eigen::Matrix<double, 4, 4> &m );
    
    void transformStamped2Eigen( const geometry_msgs::TransformStampedPtr, Eigen::Matrix<double, 4, 4> & );
    
    void cv2Eigen( const cv::Matx44d &, Eigen::Matrix<double, 4, 4> & );
  
  protected:
    Eigen::Matrix<double, 4, 4> tfWorldSensor;
    geometry_msgs::TransformStampedPtr stamped_tf_;
  };
  
  class LaserMeasurement : public Measurement
  {
  protected:
    sensor_msgs::LaserScan laser;
    std::vector<Contour::Beam> beams_;
  
  public:
    LaserMeasurement( const geometry_msgs::TransformStampedPtr _tf );
    
    ~LaserMeasurement()
    {
    }
    
    void initFromScan( const sensor_msgs::LaserScan &_scan );
    
    const sensor_msgs::LaserScan &getLaser() const;
    
    void push_back( const Contour::Beam &_beam );
    
    void resize( const size_t _sz );
    
    const size_t size() const
    {
      return beams_.size();
    }
    
    size_t size()
    {
      return beams_.size();
    }
    
    std::vector<Contour::Beam>::iterator begin();
    
    std::vector<Contour::Beam>::iterator end();
    
    const Contour::Beam &operator[]( const size_t _sz ) const;
    
    Contour::Beam &operator[]( const size_t _sz );
    
    void clear();
    
    double max_reading_;
    double min_reading_;
  };
  
  class ImageMeasurement : public Measurement
  {
  
  public:
    
    ImageMeasurement( const cv_bridge::CvImageConstPtr &, const geometry_msgs::TransformStampedPtr );
    
    ImageMeasurement( const geometry_msgs::TransformStampedPtr &_tf, const cv_bridge::CvImagePtr &image );
    
    ImageMeasurement( const cv_bridge::CvImagePtr &, const geometry_msgs::TransformStampedPtr,
                      const sensor_msgs::CameraInfo & );
    
    ImageMeasurement( const cv_bridge::CvImagePtr &_image, const geometry_msgs::TransformStampedPtr _tf );
    
    ~ImageMeasurement()
    {
    }
    
    cv_bridge::CvImagePtr &getImage();
    
    cv::Mat &cv();
    
    const std::shared_ptr<image_geometry::PinholeCameraModel> &getCameraModel() const;
    
    std::shared_ptr<image_geometry::PinholeCameraModel> &getCameraModel();
    
    void setImage( const cv_bridge::CvImagePtr &image );
    
    void setCameraModel( const std::shared_ptr<image_geometry::PinholeCameraModel> &camera_model_ptr );
    
    void preserve();
  
  protected:
    cv_bridge::CvImagePtr image;
    cv::Mat copy_img_;
    std::shared_ptr<image_geometry::PinholeCameraModel> camera_;
  };
};

#endif
