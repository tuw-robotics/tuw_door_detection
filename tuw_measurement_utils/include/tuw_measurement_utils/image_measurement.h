//
// Created by felix on 20.01.19.
//

#ifndef TUW_IMAGE_MEASUREMENT_H
#define TUW_IMAGE_MEASUREMENT_H

#include <tuw_measurement_utils/measurement.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

namespace tuw
{

  class ImageMeasurement : public Measurement
  {

  public:

    ImageMeasurement( const cv_bridge::CvImageConstPtr &, const geometry_msgs::TransformStampedPtr & );

    ImageMeasurement( const cv_bridge::CvImagePtr &, const geometry_msgs::TransformStampedPtr &,
                      const sensor_msgs::CameraInfo & );

    ImageMeasurement( const cv_bridge::CvImagePtr &_image, const geometry_msgs::TransformStampedPtr &_tf );

    ~ImageMeasurement() = default;

    cv_bridge::CvImagePtr &getImage();

    cv::Mat &cv();

    cv::Mat &getOriginalImage();

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

}

#endif //PROJECT_IMAGE_MEASUREMENT_H
