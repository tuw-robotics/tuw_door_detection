//
// Created by felix on 13.11.18.
//

#include "tuw_measurement_utils/measurements.h"
#include <tuw_geometry/utils.h>

using namespace tuw;

//@ToDo delete
void
Measurement::vectorQuaternionToEigen( const tf::Point &pt, const tf::Quaternion &q, Eigen::Matrix<double, 4, 4> &m ) {
  Eigen::Quaterniond qe( q.w(), q.x(), q.y(), q.z());
  Eigen::Vector3d v( pt.x(), pt.y(), pt.z());
  
  m.setIdentity();
  m.topLeftCorner<3, 3>() = qe.toRotationMatrix();
  m.topRightCorner<3, 1>() = v;
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

Measurement::Measurement( const tf::StampedTransform &_tf ) {
  setTfWorldSensor( _tf );
}

const Eigen::Matrix<double, 4, 4> &Measurement::getTfWorldSensor() const {
  return tfWorldSensor;
}

void Measurement::setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor ) {
  Measurement::tfWorldSensor = tfWorldSensor;
}

void Measurement::setTfWorldSensor( cv::Mat &_tf ) {
  cv2Eigen( _tf, tfWorldSensor );
}

void Measurement::setTfWorldSensor( const tf::StampedTransform &_tf ) {
  vectorQuaternionToEigen( _tf.getOrigin(), _tf.getRotation(), tfWorldSensor );
}

//@ToDo: segfault??
//LaserMeasurement::LaserMeasurement( const sensor_msgs::LaserScanConstPtr _laser, const tf::StampedTransform &_tf )
//    : Measurement( _tf ) {
//  laser = sensor_msgs::LaserScan( *_laser.get());
//}

LaserMeasurement::LaserMeasurement( const sensor_msgs::LaserScan &_laser,
                                    const tf::StampedTransform &_tf ) : laser( _laser ), Measurement( _tf ) {
  
}

void LaserMeasurement::initFromScan() {
  beams_.clear();
  std::size_t n = laser.ranges.size();
  std::cout << "laser ranges size " << n << std::endl;
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

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImageConstPtr &_image, const tf::StampedTransform &_tf )
    : Measurement( _tf ) {
  image.reset( new cv_bridge::CvImage( *_image.get()));
}

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image, const tf::StampedTransform &_tf,
                                    const sensor_msgs::CameraInfo &_camInfo ) : image(
    _image ), Measurement( _tf ) {
  camera_.reset( new image_geometry::PinholeCameraModel());
  camera_->fromCameraInfo( _camInfo );
}


ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image, const tf::StampedTransform &_tf ) : image(
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
