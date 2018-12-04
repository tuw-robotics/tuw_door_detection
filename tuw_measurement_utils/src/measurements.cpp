//
// Created by felix on 13.11.18.
//

#include "tuw_measurement_utils/measurements.h"
#include <tuw_geometry/utils.h>

using namespace tuw;

void
Measurement::transformStamped2Eigen( const geometry_msgs::TransformStampedPtr tf, Eigen::Matrix<double, 4, 4> &mat ) {
  auto ros_q = tf->transform.rotation;
  auto ros_t = tf->transform.translation;
  
  Eigen::Quaterniond q;
  q.x() = ros_q.x;
  q.y() = ros_q.y;
  q.z() = ros_q.z;
  q.w() = ros_q.w;
  
  Eigen::Vector3d t( ros_t.x, ros_t.y, ros_t.z );
  
  mat.setIdentity();
  mat.topLeftCorner<3, 3>() = q.toRotationMatrix();
  mat.topRightCorner<3, 1>() = t;
}

void Measurement::cv2Eigen( const cv::Matx44d &_we_dont_want, Eigen::Matrix<double, 4, 4> &_we_want ) {
  for ( int i = 0; i < _we_dont_want.rows; ++i ) {
    for ( int j = 0; j < _we_dont_want.cols; ++j ) {
      _we_want( i, j ) = _we_dont_want( i, j );
    }
  }
}

Measurement::Measurement( const Eigen::Matrix<double, 4, 4> &tfWorldSensor ) : tfWorldSensor( tfWorldSensor ) {
}

Measurement::Measurement( const geometry_msgs::TransformStampedPtr _tf ) {
  setTfWorldSensor( _tf );
}

const Eigen::Matrix<double, 4, 4> &Measurement::getTfWorldSensor() const {
  return tfWorldSensor;
}

void Measurement::setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor, bool override_stamped ) {
  Measurement::tfWorldSensor = tfWorldSensor;
  if ( override_stamped ) {
    Eigen::Matrix3d r = tfWorldSensor.topLeftCorner<3, 3>();
    Eigen::Quaterniond q( r );
    Eigen::Vector3d t = tfWorldSensor.topRightCorner<3, 1>();
    
    stamped_tf_->transform.translation.x = t( 0 );
    stamped_tf_->transform.translation.y = t( 1 );
    stamped_tf_->transform.translation.z = t( 2 );
    
    stamped_tf_->transform.rotation.x = q.x();
    stamped_tf_->transform.rotation.y = q.y();
    stamped_tf_->transform.rotation.z = q.z();
    stamped_tf_->transform.rotation.w = q.w();
    
  }
}

void Measurement::setTfWorldSensor( cv::Mat &_tf ) {
  cv2Eigen( _tf, tfWorldSensor );
}

const geometry_msgs::TransformStampedPtr Measurement::getStampedTf() const {
  return stamped_tf_;
}

void Measurement::setTfWorldSensor( geometry_msgs::TransformStampedPtr _tf ) {
  stamped_tf_.reset( new geometry_msgs::TransformStamped( *_tf ));
  transformStamped2Eigen( stamped_tf_, tfWorldSensor );
}

LaserMeasurement::LaserMeasurement( const geometry_msgs::TransformStampedPtr _tf ) : Measurement( _tf ) {
}

void LaserMeasurement::clear() {
  beams_.clear();
}

void LaserMeasurement::initFromScan( const sensor_msgs::LaserScan &_scan ) {
  clear();
  this->laser = _scan;
  
  std::size_t n = laser.ranges.size();
  std::size_t ii = 0;
  
  for ( ; ii < n; ++ii ) {
    double range = laser.ranges[ii];
    if ( isfinite( range ) && range < laser.range_max ) {
      const double angle = laser.angle_min + (laser.angle_increment * ii);
      const Point2D pt( cos( angle ) * range, sin( angle ) * range );
      push_back( Contour::Beam( range, angle, pt ));
    }
  }
}

const sensor_msgs::LaserScan &LaserMeasurement::getLaser() const {
  return laser;
}

void LaserMeasurement::push_back( const tuw::Contour::Beam &_beam ) {
  this->beams_.push_back( _beam );
}

void LaserMeasurement::resize( const size_t _sz ) {
  beams_.resize( _sz );
}

const Contour::Beam &LaserMeasurement::operator[]( const size_t _sz ) const {
  return beams_[_sz];
}

Contour::Beam &LaserMeasurement::operator[]( const size_t _sz ) {
  return beams_[_sz];
}

std::vector<Contour::Beam>::iterator LaserMeasurement::begin() {
  return beams_.begin();
}

std::vector<Contour::Beam>::iterator LaserMeasurement::end() {
  return beams_.end();
}

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImageConstPtr &_image,
                                    const geometry_msgs::TransformStampedPtr _tf )
    : Measurement( _tf ) {
  image.reset( new cv_bridge::CvImage( *_image.get()));
}

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image, const geometry_msgs::TransformStampedPtr _tf,
                                    const sensor_msgs::CameraInfo &_camInfo ) : image(
    _image ), Measurement( _tf ) {
  camera_.reset( new image_geometry::PinholeCameraModel());
  camera_->fromCameraInfo( _camInfo );
}


ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image, const geometry_msgs::TransformStampedPtr _tf )
    : image(
    _image ), Measurement( _tf ) {
  camera_.reset( new image_geometry::PinholeCameraModel());
}

void ImageMeasurement::setCameraModel( const std::shared_ptr<image_geometry::PinholeCameraModel> &_cam ) {
  camera_ = _cam;
}

cv_bridge::CvImagePtr &ImageMeasurement::getImage() {
  return image;
}

const std::shared_ptr<image_geometry::PinholeCameraModel> &ImageMeasurement::getCameraModel() const {
  return camera_;
}

std::shared_ptr<image_geometry::PinholeCameraModel> &ImageMeasurement::getCameraModel() {
  return camera_;
}

cv::Mat &ImageMeasurement::getCVImage() {
  return image->image;
}

void ImageMeasurement::setImage( const cv_bridge::CvImagePtr &image ) {
  ImageMeasurement::image = image;
}
