#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <laserproc/contour.h>

namespace tuw {

    class Measurement {
    public:

        Measurement(const Eigen::Matrix<double, 4, 4> &tfWorldSensor);

        Measurement(const tf::StampedTransform &_tf);

        ~Measurement() {}

        const Eigen::Matrix<double, 4, 4> &getTfWorldSensor() const;

        void setTfWorldSensor(const Eigen::Matrix<double, 4, 4> &tfWorldSensor);

        void setTfWorldSensor(const tf::StampedTransform &_tf);

    protected:
        Eigen::Matrix<double, 4, 4> tfWorldSensor;
    };

    class LaserMeasurement : public Measurement {
    protected:
        sensor_msgs::LaserScan laser;
        std::vector<Contour::Beam> beams_;

    public:
        LaserMeasurement(const sensor_msgs::LaserScan &_laser, const tf::StampedTransform &_tf);

        ~LaserMeasurement() {}

        const sensor_msgs::LaserScan &getLaser() const;

        void setLaser(const sensor_msgs::LaserScan &laser);

        void push_back(const Contour::Beam &_beam);

        void resize(const size_t _sz);

        const size_t size() { return beams_.size(); }

        std::vector<Contour::Beam>::iterator begin();

        std::vector<Contour::Beam>::iterator end();

        const Contour::Beam &operator[](const size_t _sz) const;

        Contour::Beam &operator[](const size_t _sz);
    };

    class ImageMeasurement : public Measurement {
    protected:
        cv_bridge::CvImagePtr image;

    public:
        ImageMeasurement(const cv_bridge::CvImagePtr &image, const tf::StampedTransform &_tf);

        ~ImageMeasurement() {}

        cv_bridge::CvImagePtr &getImage();

        void setImage(const cv_bridge::CvImagePtr &image);

    };
};

#endif
