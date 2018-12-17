#include "laserproc/door_depth_detector.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <tuw_geometry/figure.h>
#include <tuw_geometry/point2d.h>
#include <tuw_measurement_utils/contour.h>
#include <random>

using namespace tuw;
using namespace tuw::door_laser_proc;

DoorDepthDetector::ParametersNode::ParametersNode() : node("~")
{
  node.param<std::string>("source_frame", source_frame, "/base_link");
  node.param<std::string>("world_frame", world_frame, "/map");
  node.param<bool>("debug", debug, false);
  node.param<std::string>("publisher_topic", publisher_topic, "/door_detections");
  node.param<std::string>("internal_mode", internal_mode, "contour");
}

DoorDepthDetector::DoorDepthDetector(ros::NodeHandle &_nh) : params_(new DoorDepthDetector::ParametersNode()),
                                                             DoorDetectorBase(_nh),
                                                             figure_local_("KernelResponses")
{
  init(params_->publisher_topic);
  config_.reset(new Config());

  figure_local_.init(config_->map_pix_x, config_->map_pix_y,
                     -2, 25,
                     config_->map_min_y, config_->map_max_y,
                     config_->map_rotation + M_PI / 2.0,
                     config_->map_grid_x, config_->map_grid_y);

  ws_map_.init(config_->map_pix_x, config_->map_pix_y,
               -2, 25,
               config_->map_min_y, config_->map_max_y,
               config_->map_rotation + M_PI / 2.0);

  std::mt19937 gen(19937);
  std::uniform_int_distribution<> dist(20, 255);

  for (size_t i = 0; i < 1000; ++i)
  {
    colorMap_[i] = cv::Scalar(dist(gen), dist(gen), dist(gen));
  }
}

DoorDepthDetector::~DoorDepthDetector() = default;

//void DoorDepthDetector::callbackConfig(tuw_door_detection::DepthDetectorConfig &_config, uint32_t level)
//{
//	config_.reset(new tuw_door_detection::DepthDetectorConfig(_config));
//}

void DoorDepthDetector::plot(const std::vector<DoorDetectionPtr> &_detections)
{
  figure_local_.clear();

  for (const auto d : _detections)
  {
    double radius = 1.0;
    cv::Scalar color;
    if (d->validDetection())
    {
      radius = 1.0 + (d->responseNormalized() * 2.0);
      color = cv::Scalar(255.0 * d->responseNormalized(), 0, 0);
    } else
    {
      color = cv::Scalar(0, 0, 0);
    }
    std::cout << *d << std::endl;
    figure_local_.circle(d->operator[](0).point, radius, color, 1.0);
  }
  cv::imshow(figure_local_.title(), figure_local_.view());
  cv::waitKey(1);
}

void DoorDepthDetector::plot(const std::vector<double> &_responses)
{
  cv::Mat plotResponses;
  std::vector<double> resp_copy(_responses);
  cv::Mat responseMat(resp_copy);
  responseMat.convertTo(responseMat, CV_64F);
  cv::Ptr<cv::plot::Plot2d> plot = cv::plot::createPlot2d(responseMat);
  plot->render(plotResponses);
  cv::imshow("plot", plotResponses);
}

template<typename T>
std::vector<T> DoorDepthDetector::normalize(std::vector<T> &_v)
{
  std::vector<T> c(_v);
  T min_resp = std::numeric_limits<T>::max();
  T max_resp = -std::numeric_limits<T>::max();

  std::cout << std::endl << std::endl;
  for (const T r : c)
  {
    //std::cout << r << ", ";
    min_resp = std::min(r, min_resp);
    max_resp = std::max(r, max_resp);
  }
  std::cout << std::endl << "min: " << min_resp << std::endl;
  std::cout << "max: " << max_resp << std::endl;

  auto it = c.begin();
  for (; it != c.end(); ++it)
  {
    *it = ((*it) - min_resp) / (max_resp - min_resp);
  }

  return std::move(c);
}

std::vector<std::shared_ptr<tuw::Contour>> DoorDepthDetector::contourMode(const sensor_msgs::LaserScan &_scan)
{

  double contour_thresh = 0.45;
  cv::Mat img = cv::Mat::zeros(ws_map_.height(), ws_map_.width(), CV_8UC3);
  cv::Mat corner_img = cv::Mat::zeros(ws_map_.height(), ws_map_.width(), CV_8U);
  const size_t N = _scan.ranges.size();
  size_t contour_cnt = 1;

  size_t i = 0;
  double last_range = _scan.ranges[0];
  size_t last_range_idx = 0;
  size_t last_angle = _scan.angle_min;
  while (!isfinite(last_range) && i < N)
  {
    i++;
    last_range = _scan.ranges[i];
    last_angle = _scan.angle_min + (_scan.angle_increment * i);
    last_range_idx = i;
  }

  std::vector<std::shared_ptr<Contour>> contours;
  contours.push_back(std::make_shared<Contour>());

  for (size_t i = last_range_idx; i < N; i++)
  {

    if (std::isfinite(_scan.ranges[i]) && _scan.ranges[i] < _scan.range_max)
    {

      Eigen::Vector2d v = range2Eigen(_scan, last_range_idx);
      Eigen::Vector2d vnext = range2Eigen(_scan, i);
      Point2D pt = Point2D(v[0], v[1]);
      Point2D ptnext = Point2D(vnext[0], vnext[1]);
      const double angle = _scan.angle_min + (_scan.angle_increment * i);

      if (abs(_scan.ranges[i] - last_range) < 0.25)
      {
        //const int color_idx = jet_sampler[contour_cnt % jet_sampler.size()];
        contours.back()->push_back(Contour::Beam::make_beam(_scan.ranges[i], angle, ptnext));
      } else
      {
        contour_cnt += 1;
        const auto beam = Contour::Beam::make_beam(_scan.ranges[i], angle, ptnext);
        contours.push_back(std::make_shared<Contour>());
        contours.back()->push_back(beam);
      }

      last_range = _scan.ranges[i];
      last_angle = angle;
      last_range_idx = i;
    }
  }

  contours.erase(std::remove_if(contours.begin(),
                                contours.end(),
                                [contour_thresh](std::shared_ptr<Contour> &c)
                                {
                                  return c->length() < contour_thresh;
                                }),
                 contours.end());

  contour_cnt = 0;
  size_t potential_doors = 0;
  for (auto &elem : contours)
  {
    contour_cnt += 1;
    cv::Scalar color = colorMap_[contour_cnt];

    elem->renderInternal(ws_map_);
    elem->cvDetectCorners();
    //elem->cvConvexityDefects(ws_map_);
    if (elem->length() > 0.6 && elem->length() < 1.2)
    {
      potential_doors++;
      elem->set_door_candidate(true);
    }
    elem->render(ws_map_, img, color, 2, true);
  }
  if (potential_doors)
  {
    std::cout << potential_doors << " potential doors " << std::endl;
  }

  cv::imshow("lines_as_img", img);
  cv::waitKey(1);

  return std::move(contours);
  //cv::imshow("corners", corner_img);
}

//Border mode only same supported
bool DoorDepthDetector::kernelMode(const sensor_msgs::LaserScan &_scan, std::vector<DoorDetectionPtr> &_detections)
{
  assert (KERNEL_SIZE % 2 == 0);

  std::vector<double> ranges;
  ranges.resize(_scan.ranges.size());
  std::copy(_scan.ranges.begin(), _scan.ranges.end(), ranges.begin());

  std::vector<double> kernel(KERNEL_SIZE);
  size_t half_size = static_cast<std::size_t>(KERNEL_SIZE / 2.0);

  //TODO: preallocate
  //Same border mode
  for (size_t i = 0; i < KERNEL_SIZE; ++i)
  {
    if (i < half_size)
    {
      kernel[i] = -1.0f;
      //slow
      ranges.insert(ranges.begin(), ranges[0]);
    } else
    {
      kernel[i] = 1.0f;
      ranges.push_back(ranges.back());
    }
  }
  size_t N = ranges.size() - half_size;
  std::vector<double> responses;

  for (size_t i = half_size; i < N; ++i)
  {
    double length = ranges[i];

    double sum = 0.0;
    size_t total_inner_loop = KERNEL_SIZE;
    for (int j = 0; j < KERNEL_SIZE; ++j)
    {
      double curr_range = ranges[(i - half_size) + j];
      //HOW TO DEAL WITH THIS APPROPRIATELY?
      if (!isfinite(curr_range) || length >= _scan.range_max)
      {
        total_inner_loop--;
        continue;
      }
      sum += (kernel[j] * curr_range);
    }
    sum /= static_cast<double>(total_inner_loop);
    if (!isfinite(sum))
    {
      sum = 0;
    }
    responses.push_back(sum);
  }

  plot(responses);
  std::vector<double> responses_normalized = normalize(responses);

  _detections.clear();
  N = _scan.ranges.size();
  for (int i = 0; i < N; ++i)
  {
    if (std::isfinite(_scan.ranges[i]) && _scan.ranges[i] < _scan.range_max)
    {
      Eigen::Vector2d point_meas = range2Eigen(_scan, i);
      auto d = std::make_shared<DoorDetection>();
      d->resize(_scan.ranges.size());
      d->operator[](0).point = Point2D(point_meas.x(), point_meas.y());
      d->operator[](0).length = _scan.ranges[i];
      d->response() = responses[i];
      d->responseNormalized() = responses_normalized[i];
      d->validDetection() = false;
      if (d->responseNormalized() > 0.7 || d->responseNormalized() < 0.2)
      {
        d->validDetection() = true;
        if (d->responseNormalized() < 0.2)
        {
          d->responseNormalized() = 1.0 - d->responseNormalized();
        }
        if (!_detections.empty())
        {
          _detections.back()->link(_detections.back(), d);
        }
        _detections.push_back(d);
      }
    }
  }
  plot(_detections);
}

bool
DoorDepthDetector::structureMode(const sensor_msgs::LaserScan &_scan, std::vector<DoorDetectionPtr> &_detections)
{
  //Eigen::Matrix<double,4,4> tf_laser2world = tf2EigenMat(tf_laser);
  //std::cout << "robot pose: (" << tf_laser2world(0,3) << ", " << tf_laser2world(1,3) << ")" << std::endl;

  size_t N = _scan.ranges.size();

  int i = 0;
  double last_range = _scan.ranges[0];
  int last_range_idx = 0;
  while (!isfinite(last_range) && i < N)
  {
    i++;
    last_range = _scan.ranges[i];
    last_range_idx = i;
  }

  std::vector<Eigen::Vector2d> detected_door_ranges;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> candidates;
  std::vector<std::pair<unsigned int, unsigned int>> candidates_idx;

  for (; i < N; ++i)
  {
    double length = _scan.ranges[i];
    double abs_diff = fabs(length - last_range);

    if ((length < _scan.range_max) && isfinite(length) && abs_diff > thresh_)
    {
      std::cout << "range diff: " << abs_diff << std::endl;
      std::cout << "range: " << length << std::endl;
      std::cout << "last range: " << last_range << std::endl;

      Eigen::Vector2d meas1 = range2Eigen(_scan, i);
      Eigen::Vector2d meas0 = range2Eigen(_scan, last_range_idx);

      candidates_idx.push_back(std::make_pair(last_range_idx, i));
      candidates.push_back(std::make_pair(meas0, meas1));
    }

    if (isfinite(length))
    {
      last_range = length;
      last_range_idx = i;
    }
  }

  std::size_t idx_start = candidates_idx[0].second;
  std::size_t idx_candidates = 1;
  for (; idx_candidates < candidates_idx.size(); ++idx_candidates)
  {
  }

  //
  std::set<std::size_t> idx_set;
  i = 0;
  for (const auto &c : candidates_idx)
  {
    if (idx_set.count(c.first) == 0)
    {
      auto d = std::make_shared<DoorDetection>();
      d->resize(1);
      d->operator[](0).point = Point2D(candidates[i].first.x(), candidates[i].first.y());
      d->validDetection() = true;
      _detections.push_back(d);
      idx_set.insert(c.first);
    }

    if (idx_set.count(c.second) == 0)
    {
      auto d = std::make_shared<DoorDetection>();
      d->operator[](0).point = Point2D(candidates[i].second.x(), candidates[i].second.y());
      d->validDetection() = true;
      _detections.push_back(d);
      idx_set.insert(c.second);
    }
    i++;
  }
}

bool DoorDepthDetector::processLaser(const sensor_msgs::LaserScan &_scan)
{

  last_header_.reset(new std_msgs::Header(_scan.header));
  objects_.clear();

  tf::StampedTransform tf_laser;
  if (!getTF(params()->world_frame, _scan.header.frame_id, tf_laser, params()->debug))
  {
    return false;
  }

  door_detection_.clear();
  if (params()->internal_mode == "structure")
  {
    structureMode(_scan, door_detection_);
  } else if (params()->internal_mode == "kernel")
  {
    kernelMode(_scan, door_detection_);
  } else if (params()->internal_mode == "contour")
  {
    contours_ = contourMode(_scan);
  } else
  {
    ROS_ERROR("Mode not supported, cant process laser");
  }

  //t = tf_laser2world * t;
  for (DoorDetectionPtr d : door_detection_)
  {
    if (!d->validDetection())
    {
      //TODO: cleanup
      continue;
    }
    Eigen::Quaterniond q(1, 0, 0, 0);
    tuw_object_msgs::ObjectWithCovariance out_obj;
    const auto point = d->operator[](0).point;

    out_obj.object.pose.position.x = point.x();
    out_obj.object.pose.position.y = point.y();
    out_obj.object.pose.position.z = 0;//v[2];

    out_obj.object.pose.orientation.x = q.x();
    out_obj.object.pose.orientation.y = q.y();
    out_obj.object.pose.orientation.z = q.z();
    out_obj.object.pose.orientation.w = q.w();

    out_obj.object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
    out_obj.object.ids = {
        static_cast<int>(objects_.size())
    };
    out_obj.object.ids_confidence = {0};
    out_obj.object.shape_variables = {0.1, 1, 0.5};
    objects_.push_back(std::move(out_obj));
  }

  return true;
}


