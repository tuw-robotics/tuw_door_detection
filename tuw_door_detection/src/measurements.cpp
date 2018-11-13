//
// Created by felix on 13.11.18.
//

#include <measurements.h>
#include <tuw_geometry/utils.h>

using namespace tuw;

Measurement::Measurement(const Eigen::Matrix<double, 4, 4> &tfWorldSensor) : tfWorldSensor(tfWorldSensor) {}

Measurement::Measurement(const tf::StampedTransform &_tf) {
  setTfWorldSensor(_tf);
}

const Eigen::Matrix<double, 4, 4> &Measurement::getTfWorldSensor() const {
  return tfWorldSensor;
}

void Measurement::setTfWorldSensor(const Eigen::Matrix<double, 4, 4> &tfWorldSensor) {
  Measurement::tfWorldSensor = tfWorldSensor;
}

void Measurement::setTfWorldSensor(const tf::StampedTransform &_tf) {
  vectorQuaternionToEigen(_tf.getOrigin(), _tf.getRotation(), tfWorldSensor);
}

LaserMeasurement::LaserMeasurement(const sensor_msgs::LaserScan &_laser,
                                   const tf::StampedTransform &_tf) : laser(_laser), Measurement(_tf) {

}

const sensor_msgs::LaserScan &LaserMeasurement::getLaser() const {
  return laser;
}

void LaserMeasurement::setLaser(const sensor_msgs::LaserScan &laser) {
  LaserMeasurement::laser = laser;
}

ImageMeasurement::ImageMeasurement(const cv_bridge::CvImagePtr &_image, const tf::StampedTransform &_tf) : image(
    _image), Measurement(_tf) {

}

cv_bridge::CvImagePtr &ImageMeasurement::getImage() {
  return image;
}

void ImageMeasurement::setImage(const cv_bridge::CvImagePtr &image) {
  ImageMeasurement::image = image;
}
