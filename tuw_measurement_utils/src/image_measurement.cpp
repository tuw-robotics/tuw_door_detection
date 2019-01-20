//
// Created by felix on 20.01.19.
//

#include "tuw_measurement_utils/image_measurement.h"

using namespace tuw;

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImageConstPtr &_image,
                                    const geometry_msgs::TransformStampedPtr &_tf )
    : Measurement(_tf)
{
  image.reset(new cv_bridge::CvImage(*_image.get()));
}

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image,
                                    const geometry_msgs::TransformStampedPtr &_tf,
                                    const sensor_msgs::CameraInfo &_camInfo ) :
    Measurement(_tf)
{
  image = _image;
  camera_.reset(new image_geometry::PinholeCameraModel());
  camera_->fromCameraInfo(_camInfo);
}


ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image, const geometry_msgs::TransformStampedPtr &_tf )
    : image(
    _image), Measurement(_tf)
{
  camera_.reset(new image_geometry::PinholeCameraModel());
}

void ImageMeasurement::setCameraModel( const std::shared_ptr<image_geometry::PinholeCameraModel> &_cam )
{
  camera_ = _cam;
}

cv_bridge::CvImagePtr &ImageMeasurement::getImage()
{
  return image;
}

const std::shared_ptr<image_geometry::PinholeCameraModel> &ImageMeasurement::getCameraModel() const
{
  return camera_;
}

std::shared_ptr<image_geometry::PinholeCameraModel> &ImageMeasurement::getCameraModel()
{
  return camera_;
}

cv::Mat &ImageMeasurement::getOriginalImage()
{
  return copy_img_;
}

cv::Mat &ImageMeasurement::cv()
{
  return image->image;
}

void ImageMeasurement::setImage( const cv_bridge::CvImagePtr &image )
{
  ImageMeasurement::image = image;
}

void ImageMeasurement::preserve()
{
  image->image.copyTo(copy_img_);
}
