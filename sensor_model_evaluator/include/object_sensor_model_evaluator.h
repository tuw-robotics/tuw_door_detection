//
// Created by felix on 11.04.19.
//

#ifndef TUW_DOOR_DETECTION_OBJECT_SENSOR_MODEL_EVALUATOR_H
#define TUW_DOOR_DETECTION_OBJECT_SENSOR_MODEL_EVALUATOR_H

#include <tuw_object_msgs/ObjectDetection.h>
#include <datastructures/octo_object_map.h>
#include <Eigen/Core>
#include <map>
#include <memory>
#include <boost/filesystem.hpp>
#include <laser_scan_contour.h>

namespace tuw
{
  class ObjectSensorModel
  {
  public:
    
    using Ptr = std::shared_ptr<ObjectSensorModel>;
  
    struct ObsExp
    {
    private:
      std::shared_ptr<Eigen::Vector3d> obs_pos;
      std::shared_ptr<Eigen::Vector3d> exp_pos;
      Eigen::Matrix4d tf_world_robot;
    
    public:
    
      ObsExp( const Eigen::Matrix4d &tf ) : obs_pos( nullptr ), exp_pos( nullptr )
      {
        tf_world_robot = tf;
      }
    
      ~ObsExp() = default;
    
      Eigen::Matrix4d &tf()
      {
        return tf_world_robot;
      }
    
      bool obsOnly()
      {
        return obs_pos && !exp_pos;
      }
      
      void obs( Eigen::Vector3d &obs )
      {
        obs_pos = std::make_shared<Eigen::Vector3d>( obs );
      }
    
      void exp( Eigen::Vector3d &exp )
      {
        exp_pos = std::make_shared<Eigen::Vector3d>( exp );
      }
    
      std::shared_ptr<Eigen::Vector3d> &obs()
      {
        return obs_pos;
      }
    
      std::shared_ptr<Eigen::Vector3d> &exp()
      {
        return exp_pos;
      }
    };
  
    struct Result
    {
    public:
      Result() = default;
    
      ~Result() = default;
    
      std::shared_ptr<ObsExp> obs_exp_data;
      float dist;
      float angular_dist;
    
      std::string asCsv()
      {
        return std::to_string( dist ) + ", " + std::to_string( angular_dist ) + "\n";
      }
    };
  
    struct Results
    {
    public:
      Results()
      {
        false_positives = 0;
        true_positives = 0;
        total_observations = 0;
      }
      
      std::vector<Result> data;
      uint64_t false_positives;
      uint64_t true_positives;
      uint64_t total_observations;
    
      std::string asCsv()
      {
        std::string csv_str( "" );
        csv_str = csv_str + std::to_string( false_positives ) + ", " + std::to_string( true_positives ) + ", " +
                  std::to_string( total_observations ) + "\n";
        for ( Result &r : data )
        {
          csv_str += r.asCsv();
        }
        return csv_str;
      }
    };
    
    ObjectSensorModel();
    
    ~ObjectSensorModel() = default;

    void processLaser(LaserScanContour::Ptr contour);

    /**
     * If msg type is SHAPE_MAP_DOOR then the doors location is stored in a pcl octree.
     * If msg type is SHAPE_DOOR the the msg is treated as an actual observation and its nearest neighbor in the door map is searched
     *
     * @param msg Object message, in particular SHAPE_MAP_DOOR and SHAPE_DOOR are supported
     * @param tf The tf from laser space into world space
     */
    void process( const tuw_object_msgs::ObjectWithCovariance &msg, const Eigen::Matrix4d &tf );

    /**
     * An iteration is defined as the processing of single ObjectDetection msg, that contains all seen objects
     * This method must be called before the process method for each individual object.
     */
    void beginIteration();

    /**
     * Concludes the iteration, must be called after processing ALL objects with process()
     */
    void endIteration();
    
    /**
     * Evaluates the processed messages i.e. distance euclidean and angular computation
     */
    void evaluate();

    void printHitMissRatio();
  
    void serializeResult( const std::string &filename, const bool continuous );

    /**
     * clears internal datastructures and returns true if data has been written and false if not
     * @return
     */
    bool clear();
  
  private:
    class IterationData
    {
    public:
      IterationData()
      {
        tp = 0;
        fp = 0;
        tn = 0;
        fn = 0;
        predicted_condition_positive = 0;
      }

      int tp;
      int fp;
      int tn;
      int fn;
      int predicted_condition_positive;
    };
    std::shared_ptr<OctoObjectMap> octo_object_map_;

    //Mirror of octo object map for contour map determination TODO: performance optimization
    std::vector<Eigen::Vector3d> mirror_object_map_os_;
    std::vector<Eigen::Vector3d> mirror_object_map_ws_;

    //This is used for finding unseen doors that could be detected in the laser (False negatives)
    std::vector<Eigen::Vector3d> doors_in_view_;
    std::vector<IterationData> iteration_data_;
    LaserScanContour::Ptr current_laser_contour_for_vis_;

    //Stores the current number of received laser scan measurements (triggers for object detections)
    std::vector<IterationData> iterations_;
    bool iteration_invalid_;
    IterationData tp_fp_total_;

    uint64_t hit_counter_;
    uint64_t miss_counter_;
    double distance_threshold_ = 1.1;

    bool data_is_written_;
    
    /**
     * contains the observed and expected measurements collected during a @see ObjectSensorModel::process call.
     */
    std::vector<std::shared_ptr<ObsExp>> obs_exp_table_;
    
    /**
     * contains the results from an evaluate call
     */
    std::shared_ptr<Results> results_;
  
    void internal_serializeResult( boost::filesystem::ofstream &of );
  };
  
  using ObjectSensorModelPtr = std::shared_ptr<ObjectSensorModel>;
}


#endif //SRC_OBJECT_SENSOR_MODEL_EVALUATOR_H
