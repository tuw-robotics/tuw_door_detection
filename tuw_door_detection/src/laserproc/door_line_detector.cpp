#include "laserproc/door_line_detector.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Core>

using namespace tuw;

DoorLineDetector::DoorLineDetector(ros::NodeHandle &_nh) : DoorDetector(_nh) {
  reconfigure_fnc_ = boost::bind(&DoorLineDetector::callbackConfig, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_fnc_);
}

DoorLineDetector::~DoorLineDetector() {
}

void DoorLineDetector::callbackConfig(tuw_geometry::Linesegment2DDetectorConfig &config, uint32_t level) {
  config_.threshold_split_neighbor = config.line_dection_split_neighbor;
  config_.threshold_split = config.line_dection_split_threshold;
  config_.min_length = config.line_dection_min_length;
  config_.min_points_per_line = config.line_dection_min_points_per_line;
  config_.min_points_per_unit = config.line_dection_min_points_per_unit;
  door_range.first = 0.6;
  door_range.second = 1.0;
}

bool DoorLineDetector::processLaser(const sensor_msgs::LaserScan &_laser) {
  //ROS_INFO("callback laser");
  sensor_msgs::LaserScan output_scan = sensor_msgs::LaserScan(_laser);
  int nr = (_laser.angle_max - _laser.angle_min) / _laser.angle_increment;
  measurement_laser_->range_max() = _laser.range_max;
  measurement_laser_->range_min() = _laser.range_min;
  measurement_laser_->resize(nr);
  measurement_laser_->stamp() = _laser.header.stamp.toBoost();
  for (int i = 0; i < nr; i++) {
    MeasurementLaser::Beam &beam = measurement_laser_->operator[](i);
    beam.length = _laser.ranges[i];
    beam.angle = _laser.angle_min + (_laser.angle_increment * i);
    beam.end_point.x() = cos(beam.angle) * beam.length;
    beam.end_point.y() = sin(beam.angle) * beam.length;
  }

  measurement_local_scanpoints_.resize(measurement_laser_->size());
  for (size_t i = 0; i < measurement_laser_->size(); i++) {
    measurement_local_scanpoints_[i] = measurement_laser_->operator[](i).end_point;
  }
  measurement_linesegments_.clear();
  //start(measurement_local_scanpoints_, measurement_linesegments_);
  // publish found line segments
  tuw_object_msgs::ObjectDetection detected_doors_msg;
  detected_doors_msg.header = _laser.header;
  //detected_doors_msg.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_DOOR;
  detected_doors_msg.sensor_type = tuw_object_msgs::ObjectDetection::SENSOR_TYPE_GENERIC_LASER_2D;

  int nr_door_detections = 0;

  tuw_geometry_msgs::LineSegment line_segment_msg;
  tuw_geometry_msgs::LineSegments line_segments_msg;
  std::pair<double, double> min_xy;
  std::pair<double, double> max_xy;
  std::vector<std::pair<int, int>> laser_door_ranges;

  for (int i = 0; i < measurement_linesegments_.size(); i++) {
    line_segment_msg.p0.x = measurement_linesegments_[i].p0().x();
    line_segment_msg.p0.y = measurement_linesegments_[i].p0().y();
    line_segment_msg.p0.z = 0;
    line_segment_msg.p1.x = measurement_linesegments_[i].p1().x();
    line_segment_msg.p1.y = measurement_linesegments_[i].p1().y();
    line_segment_msg.p1.z = 0;
    line_segments_msg.segments.push_back(line_segment_msg);
    min_xy.first = std::min(min_xy.first, line_segment_msg.p0.x);
    min_xy.first = std::min(min_xy.first, line_segment_msg.p1.x);
    min_xy.second = std::min(min_xy.second, line_segment_msg.p0.y);
    min_xy.second = std::min(min_xy.second, line_segment_msg.p1.y);
    max_xy.first = std::max(max_xy.first, line_segment_msg.p0.x);
    max_xy.first = std::max(max_xy.first, line_segment_msg.p1.x);
    max_xy.second = std::max(max_xy.second, line_segment_msg.p0.y);
    max_xy.second = std::max(max_xy.second, line_segment_msg.p1.y);
    if (is_in_doorrange(line_segment_msg)) {
      tuw_object_msgs::ObjectWithCovariance obj;
      obj.covariance_pose.emplace_back(1.0);
      obj.covariance_pose.emplace_back(0);
      obj.covariance_pose.emplace_back(0);
      obj.covariance_pose.emplace_back(0);
      obj.covariance_pose.emplace_back(1.0);
      obj.covariance_pose.emplace_back(0);
      obj.covariance_pose.emplace_back(0);
      obj.covariance_pose.emplace_back(0);
      obj.covariance_pose.emplace_back(1.0);
      obj.object.ids.emplace_back(nr_door_detections);
      obj.object.shape = tuw_object_msgs::Object::SHAPE_DOOR;
      obj.object.ids_confidence.emplace_back(1.0);
      Eigen::Vector3d pose_sensor(line_segment_msg.p0.x, line_segment_msg.p0.y, 0.0);

      obj.object.pose.position.x = line_segment_msg.p0.x;
      obj.object.pose.position.y = line_segment_msg.p0.y;
      obj.object.pose.position.z = 0;
      obj.object.pose.orientation.x = 0;
      obj.object.pose.orientation.y = 0;
      obj.object.pose.orientation.z = 0;
      obj.object.pose.orientation.w = 1;
      //TODO: opening angle determination as message twist
      detected_doors_msg.objects.emplace_back(obj);
      nr_door_detections++;
      laser_door_ranges.push_back(std::pair<int, int>(measurement_linesegments_[i].idx0_,
                                                      measurement_linesegments_[i].idx1_));

    }
  }

  if (modify_laser_scan_ && laser_door_ranges.size() > 0) {
    std::sort(laser_door_ranges.begin(), laser_door_ranges.end(),
              [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
                return a.first < b.first;
              });
    std::cout << "laser door ranges: " << laser_door_ranges.size() << std::endl;
    for (int kk = 0; kk < laser_door_ranges.size(); kk++)
      std::cout << "(" << laser_door_ranges[kk].first << ", " << laser_door_ranges[kk].second << ")" << std::endl;
    int nr_current_door_range = 0;
    std::pair<int, int> current_door_range = laser_door_ranges[nr_current_door_range];
    for (int laser_idx = 0; laser_idx < _laser.ranges.size(); laser_idx++) {
      if (current_door_range.first <= laser_idx && current_door_range.second >= laser_idx) {
        if (laser_idx == current_door_range.second && nr_current_door_range < laser_door_ranges.size()) {
          nr_current_door_range++;
          current_door_range = laser_door_ranges[nr_current_door_range];
        }
        if (doors_filter_mode_ == DoorLineDetector::FilterMode::FILTER_DOORS) {
          output_scan.ranges[laser_idx] = _laser.range_max;
        }
      } else if (doors_filter_mode_ == DoorLineDetector::FilterMode::FILTER_NON_DOORS) {
        output_scan.ranges[laser_idx] = _laser.range_max;
      }
    }
    std::cout << "Doors included " << nr_current_door_range << std::endl;
  }

  ROS_INFO("publishing laser scan");
  laser_pub_.publish(output_scan);

  if (display_window_ && line_segments_msg.segments.size() > 0) {
    double scale = 100;
    min_xy.first *= scale;
    min_xy.second *= scale;
    max_xy.first *= scale;
    max_xy.second *= scale;
    double xrange;
    double yrange;
    double xnormalizer = 0;
    double ynormalizer = 0;
    if (min_xy.first < 0) {
      xnormalizer = -min_xy.first;
      xrange = -min_xy.first + max_xy.first;
    } else
      xrange = max_xy.first - min_xy.first;
    if (min_xy.second < 0) {
      ynormalizer = -min_xy.second;
      yrange = -min_xy.second + max_xy.second;
    } else
      yrange = max_xy.second - min_xy.second;

    if (xrange > 0 && yrange > 0) {
      cv::Mat line_segments_mat = cv::Mat::zeros(xrange, yrange, CV_8U);
      cv::Mat line_segments_colored = cv::Mat::zeros(xrange, yrange, CV_8UC3);

      for (int i = 0; i < line_segments_msg.segments.size(); i++) {
        auto p0 = line_segments_msg.segments[i].p0;
        auto p1 = line_segments_msg.segments[i].p1;
        cv::Point2d cv_p0((p0.y * scale) + ynormalizer, (p0.x * scale) + xnormalizer);
        cv::Point2d cv_p1((p1.y * scale) + ynormalizer, (p1.x * scale) + xnormalizer);
        if (!is_in_doorrange(line_segments_msg.segments[i])) {
          cv::line(line_segments_mat, cv_p0, cv_p1,
                   cv::Scalar(((double) i + 1.0) / ((double) line_segments_msg.segments.size() + 1.0) * 255),
                   2);
        } else {
          cv::line(line_segments_mat, cv_p0, cv_p1, cv::Scalar(255), 2);
          cv::putText(line_segments_mat, "Door", cv_p0 + cv::Point2d(0.5, 0.5), CV_FONT_HERSHEY_COMPLEX, 1.0,
                      cv::Scalar(255), 1);
        }
      }
      cv::applyColorMap(line_segments_mat, line_segments_colored, cv::COLORMAP_JET);
      std::cout << "detected lines " << line_segments_msg.segments.size() << std::endl;
      std::cout << "range (" << xrange << ", " << yrange << ")" << std::endl;
      ROS_INFO("visualizing lines");
      cv::imshow("lines", line_segments_colored);
      cv::waitKey(1);
    }
  }

  // set header information
  line_segments_msg.header = _laser.header;

  door_pub_.publish(detected_doors_msg);
  line_pub_.publish(line_segments_msg);
}

bool DoorLineDetector::is_in_doorrange(tuw_geometry_msgs::LineSegment &line_segment) {
  double dist_x = line_segment.p1.x - line_segment.p0.x;
  double dist_y = line_segment.p1.y - line_segment.p0.y;
  double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);
  return distance > door_range.first && distance < door_range.second;
}
