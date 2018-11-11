//
// Created by felix on 29.10.18.
//

#include <laserproc/contour.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

tuw::Contour::Contour() : length_(0.0) {

}

tuw::Contour::Beam::Beam(double _range, double _angle, tuw::Point2D _end_point) {
    range = _range;
    angle = _angle;
    end_point = _end_point;
}

std::shared_ptr<tuw::Contour::Beam> tuw::Contour::Beam::make_beam(double range, double angle, tuw::Point2D end_point) {
    return std::make_shared<Beam>(range, angle, end_point);
}

void tuw::Contour::push_back(std::shared_ptr<tuw::Contour::Beam> beam) {
    if (beams_.size()) {
        length_ += beams_.back()->end_point.distanceTo(beam->end_point);
    }
    beams_.push_back(beam);
}

double tuw::Contour::length() {
    if (beams_.size() <= 2) {
        return 0;
    }
    return length_;
}

void tuw::Contour::detectCorners(const size_t KERNEL_SIZE) {
    std::vector<double> sobel;
    sobel.resize(KERNEL_SIZE);

    for (size_t i = 0; i < sobel.size(); ++i) {
        if (i < KERNEL_SIZE / 2.0) {
            sobel[i] = -1;
        } else {
            sobel[i] = 1;
        }
    }

    std::vector<double> ranges;
    ranges.resize(beams_.size() + KERNEL_SIZE);
    size_t i = 0;
    for (; i < KERNEL_SIZE / 2.0; ++i) {
        ranges[i] = (*begin())->range;
    }

    for (std::vector<std::shared_ptr<Contour::Beam>>::const_iterator it_beam = begin();
         it_beam != (end() - 1); ++it_beam, ++i) {
        const auto beam = *it_beam;
        ranges[i] = beam->range;
    }

    for (; i < ranges.size(); ++i) {
        ranges[i] = (*(end() - 1))->range;
    }

}

void tuw::Contour::cvConvexityDefects(tuw::WorldScopedMaps &_map) {
    std::vector<cv::Point2i> end_points_stl;
    std::vector<int> hull;
    std::vector<cv::Vec4i> defects;
    //std::vector<cv::Vec4i> defects;

    end_points_stl.resize(beams_.size());
    for (auto i = 0; i < end_points_stl.size(); ++i) {
        const auto end_point = _map.w2m(beams_[i]->end_point);
        end_points_stl[i].x = end_point.x();
        end_points_stl[i].y = end_point.y();
    }

    cv::convexHull(end_points_stl, hull);
    cv::convexityDefects(end_points_stl, cv::Mat(hull), defects);
}

void tuw::Contour::cvDetectCorners() {
    corner_points_.clear();

    cv::Mat corners_;
    cv::cornerHarris(rendering_, corners_, 7, 3, 0.1);

    double min, max;
    cv::minMaxLoc(corners_, &min, &max);
    double thresh = max * 0.75;

    for (size_t i = 0; i < corners_.rows; ++i) {
        for (size_t j = 0; j < corners_.cols; ++j) {
            if (corners_.at<float>(j, i) > thresh) {
                const auto color = cv::Scalar(255);
                corner_points_.push_back(cv::Point2d(i, j));
                //cv::circle(corner_img, cv::Point2d(i, j), 3, color);
            }
        }
    }
}

const std::vector<cv::Point2d> &tuw::Contour::getCorners() {
    return corner_points_;
}

void tuw::Contour::renderInternal(tuw::WorldScopedMaps &map) {
    rendering_ = cv::Mat::zeros(map.height(), map.width(), CV_8U);
    cv::Scalar color_ = cv::Scalar(255);
    render(map, rendering_, color_, 2);
}

void tuw::Contour::render(tuw::WorldScopedMaps &map, cv::Mat &img, cv::Scalar &color, double rad, bool corners) {
    for (std::vector<std::shared_ptr<Contour::Beam>>::const_iterator it_beam = begin();
         it_beam != (end() - 1); ++it_beam) {
        map.line(img, (*it_beam)->end_point, (*(it_beam + 1))->end_point, color, rad);
    }

    if (corners) {
        const auto ccolor = cv::Scalar(0, 0, 255);
        for (const auto &c : corner_points_) {
            cv::circle(img, c, 3, ccolor, 1);
        }
    }
}
