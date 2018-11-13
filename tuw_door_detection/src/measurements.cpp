//
// Created by felix on 13.11.18.
//

#include <measurements.h>
#include <tuw_geometry/utils.h>

using namespace tuw;

Measurement::Measurement( const Eigen::Matrix<double, 4, 4> &tfWorldSensor ) : tfWorldSensor( tfWorldSensor ) { }

Measurement::Measurement( const tf::StampedTransform &_tf ) {
  setTfWorldSensor( _tf );
}

const Eigen::Matrix<double, 4, 4> &Measurement::getTfWorldSensor() const {
  return tfWorldSensor;
}

void Measurement::setTfWorldSensor( const Eigen::Matrix<double, 4, 4> &tfWorldSensor ) {
  Measurement::tfWorldSensor = tfWorldSensor;
}

void Measurement::setTfWorldSensor( const tf::StampedTransform &_tf ) {
  vectorQuaternionToEigen( _tf.getOrigin(), _tf.getRotation(), tfWorldSensor );
}

LaserMeasurement::LaserMeasurement( const sensor_msgs::LaserScan &_laser,
                                    const tf::StampedTransform &_tf ) : laser( _laser ), Measurement( _tf ) {
  
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

ImageMeasurement::ImageMeasurement( const cv_bridge::CvImagePtr &_image, const tf::StampedTransform &_tf,
                                    const sensor_msgs::CameraInfo &_camInfo ) : image(
    _image ), Measurement( _tf ) {
  camera_.reset( new image_geometry::PinholeCameraModel());
  camera_->fromCameraInfo( _camInfo );
}

cv_bridge::CvImagePtr &ImageMeasurement::getImage() {
  return image;
}

const std::unique_ptr<image_geometry::PinholeCameraModel> &ImageMeasurement::getCameraModel() const {
  return camera_;
}

std::unique_ptr<image_geometry::PinholeCameraModel> &ImageMeasurement::getCameraModel() {
  return camera_;
}

cv::Mat &ImageMeasurement::getCVImage() {
  return image->image;
}

void ImageMeasurement::setImage( const cv_bridge::CvImagePtr &image ) {
  ImageMeasurement::image = image;
}
