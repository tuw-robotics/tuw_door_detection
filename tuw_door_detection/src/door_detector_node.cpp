/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                         Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include "door_detector_node.h"
#include "laserproc/door_line_detector.h"
#include "laserproc/door_depth_detector.h"
#include <opencv2/highgui.hpp>


using namespace tuw;

DoorDetectorNode::ParametersNode::ParametersNode() : node( "~" )
{
  std::string mode_str;
  node.param<std::string>( "mode", mode_str, std::string( "depth" ));
  node.param<std::string>( "camera_source_frame", camera_source_frame, std::string( "" ));
  node.param<std::string>( "laser_source_frame", laser_source_frame, std::string( "" ));
  node.param<std::string>( "world_frame", world_frame, std::string( "" ));
  node.param<std::string>( "clean_laser_pub_topic", laser_pub, std::string( "clean_laser" ));
  node.param<std::string>( "mapped_doors_pub_topic", mapped_doors_topic, std::string(""));
  node.param<bool>( "debug", debug, false );
  
  try
  {
    mode = enumResolver.at( mode_str );
  }
  catch (std::exception &e)
  {
    ROS_ERROR( "ERROR: %s is not a valid detector mode.", mode_str.c_str());
  }
}

DoorDetectorNode::DoorDetectorNode() : nh_( "" ),
                                       display_window_( true ),
                                       modify_laser_scan_( true ),
                                       tf_buffer_( ros::Duration( 50, 0 )),
                                       tf_listener_( tf_buffer_ ),
                                       detection_result_( nullptr ),
                                       scan_filtered_( nullptr )
{
  
  door_detector_.reset( new DoorDetector());
  sub_laser_ = nh_.subscribe( "scan", 1000, &DoorDetectorNode::callbackLaser, this );
  sub_image_ = nh_.subscribe( "image_rgb", 1000, &DoorDetectorNode::callbackImage, this );
  sub_camera_info_rgb_ = nh_.subscribe( "camera_info_rgb", 1000, &DoorDetectorNode::callbackCameraInfoRGB, this );
  sub_camera_info_depth_ = nh_.subscribe( "camera_info_depth", 1000, &DoorDetectorNode::callbackCameraInfoDepth, this );
  sub_image_depth_ = nh_.subscribe( "image_depth", 1000, &DoorDetectorNode::callbackDepthImage, this );
  img_processor_.reset( new image_processor::DoorDetectorImageProcessor());
  
  if (params_.mapped_doors_topic != std::string(""))
  {
    pub_mapped_doors_ = nh_.advertise<tuw_object_msgs::ObjectDetection>( params_.mapped_doors_topic ,1000);
  }
  
  pub_detections_ = nh_.advertise<tuw_object_msgs::ObjectDetection>( "object_detections", 1000 );
  pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>( params_.laser_pub, 1000 );
  //line_pub_ = nh_.advertise<tuw_geometry_msgs::LineSegments>("line_segments", 1000);
  //door_pub_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("object_detection", 1000);
  //measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();
  params_ = ParametersNode();
  
  if ( params_.mode == ParametersNode::FilterMode::DEPTH )
  {
    door_detector_laser_.reset( new door_laser_proc::DoorDepthDetector( nh_ ));
  } else
  {
    door_detector_laser_.reset( new door_laser_proc::DoorLineDetector( nh_ ));
  }
  
}

DoorDetectorNode::~DoorDetectorNode() = default;

void DoorDetectorNode::callbackImage( const sensor_msgs::ImageConstPtr &_img )
{
  
  if ( !camera_info_rgb_ )
  {
    return;
  }
  
  auto image = cv_bridge::toCvCopy( _img, _img->encoding );
  cv::cvtColor( image->image, image->image, CV_RGB2GRAY );
  
  geometry_msgs::TransformStampedPtr tf;
  if ( getStaticTF( params_.world_frame, _img->header.frame_id, tf, params_.debug ))
  {
    
    //@ToDo: reset not mandatory
    image_rgb_.reset( new ImageMeasurement( image, tf, *camera_info_rgb_ ));
    
    //@ToDo: debug -> cleanup after
    image_rgb_->preserve();
    
  }
  
}

void DoorDetectorNode::callbackCameraInfoRGB( const sensor_msgs::CameraInfoConstPtr &_msg )
{
  if ( !camera_info_rgb_ )
  {
    camera_info_rgb_.reset( new sensor_msgs::CameraInfo( *_msg ));
  }
}

void DoorDetectorNode::callbackCameraInfoDepth( const sensor_msgs::CameraInfoConstPtr &_msg )
{
  if ( !camera_info_depth_ )
  {
    camera_info_depth_.reset( new sensor_msgs::CameraInfo( *_msg ));
  }
}

void DoorDetectorNode::callbackDepthImage( const sensor_msgs::ImageConstPtr &_img )
{
  
  if ( !camera_info_depth_ )
  {
    return;
  }
  
  auto image = cv_bridge::toCvCopy( _img );
  image->image.convertTo( image->image, CV_32FC1,
                          1.0 / static_cast<double>(std::numeric_limits<u_int16_t>::max()));
  
  geometry_msgs::TransformStampedPtr tf;
  if ( getStaticTF( params_.world_frame, _img->header.frame_id, tf, params_.debug ))
  {
    
    //@ToDo: reset not mandatory only image has changed
    image_depth_.reset( new ImageMeasurement( image, tf, *camera_info_depth_ ));
    
  }
  
}

void DoorDetectorNode::publish()
{
  //door_detector_laser_->publish();
  if ( detection_result_ )
  {
    pub_detections_.publish( detection_result_ );
    detection_result_ = nullptr;
  }
  if ( scan_filtered_ )
  {
    pub_laser_.publish( scan_filtered_ );
  }
  if (mapped_door_objs_)
  {
    pub_mapped_doors_.publish ( mapped_door_objs_);
    mapped_door_objs_ = nullptr;
  }
  
}

void DoorDetectorNode::callbackLaser( const sensor_msgs::LaserScan &_laser )
{
  
  geometry_msgs::TransformStampedPtr tf;
  if ( getStaticTF( params_.world_frame, _laser.header.frame_id, tf, params_.debug ))
  {
    
    laser_measurement_.reset( new LaserMeasurement( tf ));
    laser_measurement_->initFromScan( _laser );
    
  }
  
}

bool DoorDetectorNode::tryPoseFetch( Eigen::Matrix4d &tf_w_base )
{
  try
  {
    std::string world_frame = "map";
    std::string target_frame = "r0/base_link";
    
    geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
        world_frame, target_frame, ros::Time( 0 ));
    
    geometry_msgs::TransformStampedPtr stampedPtr;
    stampedPtr.reset( new geometry_msgs::TransformStamped( stamped_tf ));
    
    Measurement::transformStamped2Eigen(stampedPtr, tf_w_base);
    
  } catch (tf2::TransformException &ex)
  {
    
    ROS_INFO( "DoorDetectorNode::getStaticTF" );
    ROS_ERROR( "%s", ex.what());
    return false;
    
  }
  
  return true;
}

void DoorDetectorNode::process()
{
  if ( laser_measurement_ && image_depth_ && image_rgb_ )
  {
    auto ts_start = ros::Time::now();
    
    img_processor_->processImage( image_rgb_, image_depth_ );
    
    bool success = door_detector_laser_->processLaser( laser_measurement_->getLaser());
    
    door_detector_->setImageMeasurement( image_rgb_ );
    
    door_detector_->setLaserMeasurement( laser_measurement_ );
    
    Eigen::Matrix4d tf_w_base;
    if (tryPoseFetch(tf_w_base))
    {
      door_detector_->setRobotPosition(tf_w_base);
    }
    
    door_detector_->merge( img_processor_, door_detector_laser_ );
    
    detection_result_.reset( new tuw_object_msgs::ObjectDetection( door_detector_->getResultAsMessage()));
    
    mapped_door_objs_.reset( new tuw_object_msgs::ObjectDetection( door_detector_->getMappedDoorsAsMessage()));
    
    auto doors_in_laser = door_detector_->getDoorsLaser();
    
    std::vector<std::pair<std::size_t, std::size_t>> idxrange_;
    for ( auto dl_it = doors_in_laser.begin(); dl_it != doors_in_laser.end(); ++dl_it )
    {
      auto deref = *dl_it;
      idxrange_.push_back( std::make_pair( deref->getBB2BeamIdxs()[0], deref->getBB2BeamIdxs()[1] ));
    }
    
    scan_filtered_ = boost::make_shared<sensor_msgs::LaserScan>( laser_measurement_->filterMessage( idxrange_ ));
    
    door_detector_->display();
    
    laser_measurement_ = nullptr;
    image_depth_ = nullptr;
    image_rgb_ = nullptr;
    door_detector_->clear();
    
    auto ts_end = ros::Time::now();
    std::cout << "processing: " << (ts_end - ts_start) << "ms" << std::endl << std::endl;
  }
}

bool DoorDetectorNode::getStaticTF( const std::string &world_frame, const std::string &source_frame,
                                    geometry_msgs::TransformStampedPtr &_tf, bool debug )
{
  
  std::string _target_frame = source_frame;
  std::string _world_frame = world_frame;
  
  if ( _target_frame[0] == '/' )
  {
    _target_frame.erase( 0, 1 );
  }
  
  if ( _world_frame[0] == '/' )
  {
    _world_frame.erase( 0, 1 );
  }
  
  std::string key = _target_frame + "->" + _world_frame;
  
  if ( tfMap_.find( key ) == std::end( tfMap_ ))
  {
    try
    {
      geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
          _world_frame, _target_frame, ros::Time( 0 ));
      
      _tf.reset( new geometry_msgs::TransformStamped( stamped_tf ));
      
      tfMap_[key] = _tf;
    } catch (tf2::TransformException &ex)
    {
      
      ROS_INFO( "DoorDetectorNode::getStaticTF" );
      ROS_ERROR( "%s", ex.what());
      return false;
      
    }
  } else
  {
    _tf = tfMap_[key];
  }
  
  if ( debug )
  {
    std::cout << key << std::endl;
    const auto t = _tf->transform.translation;
    const auto q = _tf->transform.rotation;
    std::cout << "tf get o: " << t.x << ", " << t.y << ", "
              << t.z
              << std::endl;
    std::cout << "tf get r: " << q.x << ", " << q.y << ", "
              << q.z << ", " << q.w << std::endl;
  }
  
  return true;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "door_2d_detector_node" );
  
  DoorDetectorNode detector_node;
  ros::Rate rate( 30 );
  
  while ( ros::ok())
  {
    
    ros::spinOnce();
    detector_node.process();
    detector_node.publish();
    
    rate.sleep();
  }
  return 0;
}
