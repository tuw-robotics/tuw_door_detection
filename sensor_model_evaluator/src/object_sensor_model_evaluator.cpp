//
// Created by felix on 11.04.19.
//

#include <object_sensor_model_evaluator.h>
#include <opencv2/imgproc.hpp>

using namespace tuw;

ObjectSensorModel::ObjectSensorModel()
{
  octo_object_map_ = std::make_shared<OctoObjectMap>(0.5f);
  obs_exp_table_.resize(0);
  data_is_written_ = false;
  hit_counter_ = 0;
  miss_counter_ = 0;
  tp_fp_total_ = IterationData();
  iteration_invalid_ = true;
}

void ObjectSensorModel::evaluate()
{
  size_t idx = 0;
  results_ = std::shared_ptr<Results>();
  results_->total_observations = obs_exp_table_.size();

  if (obs_exp_table_.empty())
  {
    return;
  }

  for (std::shared_ptr<ObsExp> &obs_exp : obs_exp_table_)
  {
    Eigen::Vector3d &observation = *obs_exp->obs();
    Eigen::Matrix4d tf_world_laser = obs_exp->tf();
    Result result;
    result.obs_exp_data = obs_exp;

    if (obs_exp->obsOnly())
    {
      results_->false_positives++;
    } else
    {
      results_->true_positives++;
      Eigen::Vector3d &expectation = *obs_exp->exp();
      Eigen::Vector3d pnt_world_laser = tf_world_laser.topRightCorner<3, 1>();
      float dist_sqrt = (observation - expectation).norm();

      Eigen::Vector3d viewing_dir_obs = observation - pnt_world_laser;
      Eigen::Vector3d viewing_dir_exp = expectation - pnt_world_laser;
      viewing_dir_exp.normalize();
      viewing_dir_obs.normalize();
      float dist_angle = acos(viewing_dir_obs.dot(viewing_dir_exp));

      result.angular_dist = dist_angle;
      result.dist = dist_sqrt;
    }
    results_->data.push_back(std::move(result));
  }
}

bool ObjectSensorModel::clear()
{
  results_ = nullptr;
  obs_exp_table_.clear();
  hit_counter_ = 0;
  miss_counter_ = 0;
  return data_is_written_;
}

void ObjectSensorModel::printHitMissRatio()
{
  std::cout << "Hits (true positives): " << hit_counter_ << std::endl;
  std::cout << "Misses (false positives): " << miss_counter_ << std::endl;
}

void ObjectSensorModel::beginIteration()
{
  iteration_data_.push_back(IterationData());
  iteration_invalid_ = false;
}

void ObjectSensorModel::endIteration()
{
  auto &data = iteration_data_.back();
  data.predicted_condition_positive = doors_in_view_.size();
  tp_fp_total_.tp += data.tp;
  tp_fp_total_.fp += data.fp;
  tp_fp_total_.predicted_condition_positive += data.predicted_condition_positive;

  std::cout << "================================" << std::endl;
  std::cout << "TPR total: " << (float)tp_fp_total_.tp / (float)tp_fp_total_.predicted_condition_positive << std::endl;
  std::cout << "FPR total: " << (float)tp_fp_total_.fp / (float)tp_fp_total_.predicted_condition_positive << std::endl;
  std::cout << "TP total: " << tp_fp_total_.tp << std::endl;
  std::cout << "FP total: " << tp_fp_total_.fp << std::endl;
  std::cout << "Map doors: " << octo_object_map_->size() << std::endl;
  std::cout << "In sight: " << data.predicted_condition_positive << std::endl;

  iteration_invalid_ = true;

}

void ObjectSensorModel::processLaser(LaserScanContour::Ptr laser_contour)
{
  doors_in_view_.clear();

  std::cout << "process laser" << std::endl;
  std::cout << "mirror obj map size " << mirror_object_map_os_.size() << std::endl;
  std::cout << "checking doors \n ==================" << std::endl;

  for (std::size_t i = 0; i < mirror_object_map_os_.size(); i++)
  {
    auto door_os_position = mirror_object_map_os_[i];
    auto door_ws_position = mirror_object_map_ws_[i];
    double dx = door_os_position.x();
    double dy = door_os_position.y();

    //TODO: move to separate laser callback! check objects in map every time
    cv::Vec2f offset = cv::Vec2f(dx, dy) * laser_contour->scale_factor();
    offset[0] *= laser_contour->contour_offset();
    offset[1] *= laser_contour->contour_offset();
    offset = -offset;

    float dx_offset = dx * laser_contour->scale_factor() + offset[0] + laser_contour->add_factor();
    float dy_offset = dy * laser_contour->scale_factor() + offset[1] + laser_contour->add_factor();

    cv::Point2f cv_d;
    cv_d.x = dx_offset;
    cv_d.y = dy_offset;
    std::cout << "d " << i << ": (" << cv_d.x << ", " << cv_d.y << ")" << std::endl;

    if (cv::pointPolygonTest(laser_contour->getContourRepresentation(), cv_d, false) >= 0)
    {
      doors_in_view_.push_back(door_ws_position);
    }
  }

  current_laser_contour_for_vis_ = laser_contour;
  std::cout << "process laser fin" << std::endl;
  std::cout << "Doors in view: " << doors_in_view_.size() << std::endl;
}

void ObjectSensorModel::process(const tuw_object_msgs::ObjectWithCovariance &obj, const Eigen::Matrix4d &tf)
{
  if (iteration_invalid_)
  {
    ROS_ERROR("make sure to call beginIteration and endIteration, before and after process() call");
    throw new std::runtime_error("Invalid process state");
  }
  std::cout << "ObjectSensorModel is processing" << std::endl;
  std::cout << "Object shape is " << obj.object.shape << std::endl;
  using tuw_object_msgs::ObjectDetection;
  using tuw_object_msgs::Object;

  auto &door_obj = obj.object;
  auto &position = door_obj.pose.position;
  auto &orientation = door_obj.pose.orientation;

  Eigen::Matrix4d pose_mat_w;
  Eigen::Quaterniond qobject(orientation.w, orientation.x, orientation.y, orientation.z);

  pose_mat_w.topLeftCorner<3, 3>() = qobject.toRotationMatrix();
  pose_mat_w.topRightCorner<3, 1>() = Eigen::Vector3d(position.x, position.y, position.z);
  pose_mat_w = tf * pose_mat_w;

  Eigen::Quaterniond qworld(pose_mat_w.topLeftCorner<3, 3>());
  Eigen::Vector3d tworld(pose_mat_w.topRightCorner<3, 1>());

  pcl::PointXYZ search_pnt(tworld.x(), tworld.y(), tworld.z());
  Eigen::Vector3d out_pnt;

  if (obj.object.shape == Object::SHAPE_MAP_DOOR) //Door is in map -> store it
  {
    float dist_squared = -1;
    if (!octo_object_map_->searchBestPCL(search_pnt, 0.5, out_pnt, dist_squared, false))
    {
      pcl::PointXYZ pcl_pnt(tworld.x(), tworld.y(), tworld.z());
      octo_object_map_->insert(pcl_pnt);
      std::cout << "insert obj into map" << std::endl;
      mirror_object_map_os_.push_back(Eigen::Vector3d(position.x, position.y, position.z));
      mirror_object_map_ws_.push_back(tworld);
    }
  } else if (obj.object.shape == Object::SHAPE_DOOR) //Crosscheck with map
  {
    obs_exp_table_.push_back(std::unique_ptr<ObsExp>(new ObsExp(tf)));
    obs_exp_table_.back()->obs(tworld);

    data_is_written_ = false;
    float dist_squared = -1;
    if (!octo_object_map_->searchBestPCL(search_pnt, distance_threshold_, out_pnt, dist_squared, true))
    {
      //missed
      miss_counter_++;
      iteration_data_.back().fp++;
    } else
    {
      //hit -> true positives
      obs_exp_table_.back()->exp(out_pnt);
      hit_counter_++;
      iteration_data_.back().tp++;
    }
  }
}

void ObjectSensorModel::internal_serializeResult(boost::filesystem::ofstream &of)
{
  of << results_->asCsv();
  //of << std::endl;
  of.flush();
  of.close();
  data_is_written_ = true;
}

void ObjectSensorModel::serializeResult(const std::string &filepath, const bool continuous)
{
  using boost::filesystem::path;
  path p(filepath);
  if (boost::filesystem::exists(p))
  {
    if (continuous)
    {
      boost::filesystem::ofstream of(p, std::ios::app | std::ios::ate);
      internal_serializeResult(of);
    } else
    {
      boost::filesystem::ofstream of(p, std::ios::out);
      internal_serializeResult(of);
    }
    //if ( continuous_outstream_ )
    //{
    //  boost::filesystem::ofstream of( p, std::ios::app | std::ios::ate );
    //  internalSerialize( of );
    //  //of << std::endl;
    //  of.close();
    //} else if ( filesys_force_override_ )
    //{
    //  boost::filesystem::ofstream of( p, std::ios::out );
    //  internalSerialize( of );
    //  of.close();
    //}
  } else
  {
    boost::filesystem::ofstream of(p, std::ios::out);
    internal_serializeResult(of);
  }
}
