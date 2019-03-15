//
// Created by felix on 28.02.19.
//

#include <sensor_model_evaluator.h>
#include <opencv2/highgui.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tuw_measurement_utils/transforms.h>

using namespace tuw;

SensorModelEvaluator::SensorModelEvaluator( const nav_msgs::OccupancyGridConstPtr &map, bool render ) : has_result_(
    false), filesys_force_override_(true), continuous_outstream_(false)
{
  map_msg_ = nav_msgs::OccupancyGrid(*map);
  render_ = render;

  if ( !convert(map, map_))
  {
    ROS_ERROR("ERROR CONVERTING MAP TO CV");
  }

  if ( map_ && render )
  {
    render_map_ = constructDownscaled(map_, 4.0);
    render_map_->cv_uc8.copyTo(render_map_->cv_untouched_initial);
    render_map_->parent = map_;

    auto tmp = constructDownscaled(map_, 4.0);
    tmp->parent = std::move(map_);
    map_ = std::move(tmp);
    cv::cvtColor(map_->cv_uc8, map_->cv_uc8, CV_BGR2GRAY);
  }
}

void SensorModelEvaluator::configure( const sensor_model_evaluator::SensorModelEvaluatorNodeConfig &cfg )
{
  continuous_outstream_ = cfg.continuous_outstream;
}

void SensorModelEvaluator::updateObservedMeasurementTable( double angle_idx, const tuw::Point2D &obs )
{
  assert(observed_meas_.find(idx) == observed_meas_.end());
  observed_meas_.insert(std::make_pair(angle_idx, obs));
}

void SensorModelEvaluator::updateExpectedMeasurementTable( double angle_idx, const tuw::Point2D &expect )
{
  assert(expected_meas_.find(idx) == expected_meas_.end());
  expected_meas_.insert(std::make_pair(angle_idx, expect));
}

bool SensorModelEvaluator::convert( const nav_msgs::OccupancyGridConstPtr &src, std::shared_ptr<InternalMap> &des )
{
  des = std::make_unique<InternalMap>();
  ROS_INFO("map dim %d, %d\norigin: (x=%lf,y=%lf,z=%lf)",
           ( int ) src->info.width, ( int ) src->info.height,
           src->info.origin.position.x,
           src->info.origin.position.y,
           src->info.origin.position.z);

  if (( src->info.origin.orientation.x != 0 ) ||
      ( src->info.origin.orientation.y != 0 ) ||
      ( src->info.origin.orientation.z != 0 ) ||
      ( src->info.origin.orientation.w != 1 ))
  {
    ROS_WARN("UNSUPPORTED CONVERSION: Rotated map given in SensorModelEvaluator::convert()");
    ROS_WARN("(x,y,z,w) = (%lf,%lf,%lf,%lf)", src->info.origin.orientation.x,
             src->info.origin.orientation.y,
             src->info.origin.orientation.z,
             src->info.origin.orientation.w);
    return false;
  }

  des->cv_uc8 = cv::Mat::zeros(cv::Size(src->info.width, src->info.height), CV_8U);
  des->size_x = src->info.width;
  des->size_y = src->info.height;
  des->scale = src->info.resolution;
  des->origin_x = src->info.origin.position.x + (( des->size_x / 2.0 ) * des->scale );
  des->origin_y = src->info.origin.position.y + (( des->size_y / 2.0 ) * des->scale );
  des->map_info_ = src->info;

  // Convert to player format
  memcpy(des->cv_uc8.data, src->data.data(), static_cast<std::size_t>(des->size_x * des->size_y));

  return true;
}

bool SensorModelEvaluator::convert( const std::shared_ptr<InternalMap> &src, cv::Mat &des )
{
  src->cv_uc8.copyTo(des);
  return true;
}

bool SensorModelEvaluator::convert( const std::shared_ptr<InternalMap> &src, nav_msgs::OccupancyGrid &des )
{
  des.info.resolution = src->scale;
  des.info.width = src->size_x;
  des.info.height = src->size_y;
  des.info.origin = src->map_info_.origin;
  des.info.map_load_time = src->map_info_.map_load_time;

  des.data.resize(static_cast<std::size_t>(src->size_x * src->size_y));

  memcpy(des.data.data(), src->cv_uc8.data, static_cast<std::size_t>(src->size_x * src->size_y));

  return true;
}

bool SensorModelEvaluator::convert( const std::shared_ptr<tuw::SensorModelEvaluator::InternalMap> &src,
                                    grid_map_msgs::GridMap &des )
{
  nav_msgs::OccupancyGrid gm_ros;
  grid_map::GridMap gm_eth;
  convert(src, gm_ros);

  if ( !grid_map::GridMapRosConverter::fromOccupancyGrid(gm_ros, "global_map", gm_eth))
  {
    ROS_ERROR("could not convert nav_msgs::OccupancyGrid to grid_map::GridMap");
    return false;
  } else
  {
    grid_map::GridMapRosConverter::toMessage(gm_eth, des);
  }
  return true;
}

void SensorModelEvaluator::clear()
{
  expected_meas_.clear();
  observed_meas_.clear();
  render_map_->clear();
}

std::shared_ptr<SensorModelEvaluator::InternalMap>
SensorModelEvaluator::constructDownscaled( const std::shared_ptr<InternalMap> &map, const double scale_factor )
{
  std::shared_ptr<InternalMap> downscaled = std::make_shared<InternalMap>();
  map->cv_uc8.copyTo(downscaled->cv_uc8);
  downscaled->scale = map->scale * scale_factor;
  downscaled->size_x = map->size_x / scale_factor;
  downscaled->size_y = map->size_y / scale_factor;
  downscaled->map_info_ = map->map_info_;

  downscaled->origin_x = downscaled->map_info_.origin.position.x + ( downscaled->size_x / 2 ) * downscaled->scale;
  downscaled->origin_y = downscaled->map_info_.origin.position.y + ( downscaled->size_y / 2 ) * downscaled->scale;

  cv::resize(downscaled->cv_uc8, downscaled->cv_uc8, cv::Size(downscaled->size_x, downscaled->size_y));
  if ( downscaled->cv_uc8.channels() == 1 )
  {
    cv::cvtColor(downscaled->cv_uc8, downscaled->cv_uc8, CV_GRAY2BGR);
  }

  return downscaled;
}

void SensorModelEvaluator::downscaleImshow( LaserMeasurementPtr meas )
{

  cv::Point2d origin_img(render_map_->get_mx_from_wx(render_map_->origin_x),
                         render_map_->get_my_from_wy(render_map_->origin_y));

  //cv::circle( render_map_->cv_ui8, origin_img, 5, cv::Scalar( 255, 255, 0 ), 3 );

  if ( meas )
  {
    Eigen::Matrix4d tf = meas->getTfWorldSensor();
    for ( auto b_it = meas->begin();
          b_it != meas->end();
          ++b_it )
    {
      auto p_cv = b_it->end_point.cv();
      Eigen::Vector4d ws_p_b = tf * Eigen::Vector4d(b_it->end_point.x(), b_it->end_point.y(), 0, 1);
      ws_p_b = ws_p_b / ws_p_b[3];

      cv::Point2d i_p_b(render_map_->get_mx_from_wx(ws_p_b(0)),
                        render_map_->get_my_from_wy(ws_p_b(1)));

      cv::circle(render_map_->cv_uc8, i_p_b, 1, cv::Scalar(255, 0, 0), 1);
    }
  }

  cv::imshow("map", render_map_->cv_uc8);
  cv::waitKey(5);
}

void SensorModelEvaluator::evaluate( LaserMeasurementPtr &scan )
{
  clear();

  laser_meas_ = scan;
  Eigen::Matrix4d tf_ML = scan->getTfWorldSensor();
  uint32_t intersect = 0;

  auto origin_view = tf_ML.topRightCorner<2, 1>(0, 3);
  cv::Point2d origin = cv::Point2d(tf_ML(0, 3), tf_ML(1, 3));
  Eigen::Vector2d origin_eigen = Eigen::Vector2d(origin.x, origin.y);
  cv::Point2d origin_px = cv::Point2d(
      render_map_->get_mx_from_wx(tf_ML(0, 3)),
      render_map_->get_my_from_wy(tf_ML(1, 3))
  );


  cv::circle(render_map_->cv_uc8, origin_px, 2.0, cv::Scalar(0, 0, 255), 2.0);

  for ( std::vector<Beam>::iterator beam_it = scan->begin();
        beam_it != scan->end();
        ++beam_it )
  {

    Beam beam = *beam_it;
    Eigen::Vector4d end_point_w =
        scan->getTfWorldSensor() * Eigen::Vector4d(beam.end_point.x(), beam.end_point.y(), 0, 1);
    end_point_w = end_point_w / end_point_w[3];

    updateObservedMeasurementTable(beam_it->angle, Point2D(end_point_w.x(), end_point_w.y()));

    Point2DPtr intersection = rayTrace(scan->getLaser().range_max, beam, tf_ML);

    if ( intersection )
    {

      cv::circle(render_map_->cv_uc8, intersection->cv(), 1.5, cv::Scalar(0, 255, 0), 1.5);
      Point2D w_intersection(
          map_->get_wx_from_mx(intersection->x()),
          map_->get_wy_from_my(intersection->y())
      );
      updateExpectedMeasurementTable(beam_it->angle, w_intersection);

    } else
    {
      intersect++;
    }
  }

  downscaleImshow(scan);

  has_result_ = true;
}

Point2DPtr SensorModelEvaluator::rayTrace( const double scale, const Beam &beam, const Eigen::Matrix4d &tf_ML )
{
  Eigen::Vector2d origin_eigen(tf_ML(0, 3), tf_ML(1, 3));
  Eigen::Vector4d vbeamend(beam.end_point.x(), beam.end_point.y(), 0, 1);
  vbeamend = tf_ML * vbeamend;
  vbeamend = vbeamend / vbeamend(3);
  vbeamend.normalize();

  cv::Point2d dir_beam = cv::Point2d(vbeamend.x(), vbeamend.y());

  Eigen::Vector2d end_point_ws = beam.transform<Eigen::Vector2d>(tf_ML);
  Eigen::Vector2d end_vec_rs = ( end_point_ws - origin_eigen ) / ( end_point_ws - origin_eigen ).norm();
  Eigen::Vector2d end_extended = end_point_ws + ( end_vec_rs * scale );
  cv::Point2d w_range_max = cv::Point2d(end_extended.x(), end_extended.y());
  cv::Point2d w_start_hit = cv::Point2d(origin_eigen.x(), origin_eigen.y());//beam.transform<cv::Point2d>( tf_ML );

  w_start_hit.x = map_->get_mx_from_wx(w_start_hit.x);
  w_start_hit.y = map_->get_my_from_wy(w_start_hit.y);
  w_range_max.x = map_->get_mx_from_wx(w_range_max.x);
  w_range_max.y = map_->get_my_from_wy(w_range_max.y);

  cv::line(render_map_->cv_uc8, w_start_hit, w_range_max, cv::Scalar(0, 0, 255));
  cv::circle(render_map_->cv_uc8, w_range_max, 2, cv::Scalar(255, 0, 255), 2);

  cv::LineIterator ray_tracer(map_->cv_uc8, w_start_hit, w_range_max, 8);
  for ( int i = 0; i < ray_tracer.count; ++i, ++ray_tracer )
  {
    if ( map_->cv_uc8.channels() == 1 )
    {
      uchar obstacle = ( uchar ) map_->cv_uc8.at<uchar>(ray_tracer.pos());
      if ( obstacle >= 100u )
      {
        return std::make_shared<Point2D>(ray_tracer.pos().x, ray_tracer.pos().y);
      }
    } else if ( map_->cv_uc8.channels() == 3 )
    {
      ROS_ERROR("SensorModelEvaluator::rayTrace  -- multichannel map not supported --");
    }
  }
  return nullptr;
}

void SensorModelEvaluator::internalSerialize( boost::filesystem::ofstream &of )
{
  if ( !laser_meas_ )
  {
    return;
  }

  cv::Point2d origin_laser(laser_meas_->getTfWorldSensor()(0, 3),
                           laser_meas_->getTfWorldSensor()(1, 3));

  for ( auto exp = expected_meas_.begin();
        exp != expected_meas_.end();
        ++exp )
  {
    if ( observed_meas_.find(exp->first) != observed_meas_.end())
    {
      Point2D &p_obs = observed_meas_.find(exp->first)->second;
      Point2D &p_exp = exp->second;

      double obs_range = cv::norm(origin_laser - p_obs.cv());
      double exp_range = cv::norm(origin_laser - p_exp.cv());

      of << exp->first << ", " << exp_range << ", " << obs_range << ", ";
      of << p_exp.x() << ", " << p_exp.y() << ", " << p_obs.x() << ", " << p_obs.y() << "\n";
    }
  }
}

void SensorModelEvaluator::serializeResult( const std::string &filepath )
{
  using boost::filesystem::path;
  path p(filepath);
  if ( boost::filesystem::exists(p))
  {
    if ( continuous_outstream_ )
    {
      boost::filesystem::ofstream of(p, std::ios::app | std::ios::ate);
      internalSerialize(of);
      //of << std::endl;
      of.close();
    } else if ( filesys_force_override_ )
    {
      boost::filesystem::ofstream of(p, std::ios::out);
      internalSerialize(of);
      of.close();
    }
  } else if ( filesys_force_override_ )
  {
    boost::filesystem::ofstream of(p, std::ios::out);
    internalSerialize(of);
    //of << std::endl;
    of.close();
  } else
  {
    ROS_ERROR("serializeResult(): not possible with given path");
  }
}
