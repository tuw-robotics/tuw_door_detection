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

DoorDetectorNode::ParametersNode::ParametersNode() : node( "~" ) {
  std::string mode_str;
  node.param<std::string>( "mode", mode_str, std::string( "depth" ));
  node.param<std::string>( "camera_source_frame", camera_source_frame, std::string( "" ));
  node.param<std::string>( "laser_source_frame", laser_source_frame, std::string( "" ));
  node.param<std::string>( "world_frame", world_frame, std::string( "" ));
  node.param<bool>( "debug", debug, false );
  
  try {
    mode = enumResolver.at( mode_str );
  }
  catch (std::exception &e) {
    ROS_ERROR( "ERROR: %s is not a valid detector mode.", mode_str.c_str());
  }
}

DoorDetectorNode::DoorDetectorNode() : nh_( "" ), display_window_( true ), modify_laser_scan_( true ) {
  
  sub_laser_ = nh_.subscribe( "scan", 1000, &DoorDetectorNode::callbackLaser, this );
  sub_image_ = nh_.subscribe( "image_rgb", 1000, &DoorDetectorNode::callbackImage, this );
  sub_camera_info_rgb_ = nh_.subscribe( "camera_info_rgb", 1000, &DoorDetectorNode::callbackCameraInfoRGB, this );
  sub_camera_info_depth_ = nh_.subscribe( "camera_info_depth", 1000, &DoorDetectorNode::callbackCameraInfoDepth, this );
  sub_image_depth_ = nh_.subscribe( "image_depth", 1000, &DoorDetectorNode::callbackDepthImage, this );
  img_processor_.reset( new DoorDetectorImageProcessor());
  //line_pub_ = nh_.advertise<tuw_geometry_msgs::LineSegments>("line_segments", 1000);
  //door_pub_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("object_detection", 1000);
  //measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();
  params_ = ParametersNode();
  
  if ( params_.mode == ParametersNode::FilterMode::DEPTH ) {
    door_detector_.reset( new DoorDepthDetector( nh_ ));
  } else {
    door_detector_.reset( new DoorLineDetector( nh_ ));
  }
}

DoorDetectorNode::~DoorDetectorNode() {
}

void DoorDetectorNode::callbackImage( const sensor_msgs::ImageConstPtr &_img ) {
  
  if ( !camera_info_rgb_ ) {
    return;
  }
  
  auto image = cv_bridge::toCvCopy( _img, std::string( "8UC3" ));
  
  tf::StampedTransform tf;
  if ( getStaticTF( params_.world_frame, _img->header.frame_id.c_str(), tf, params_.debug )) {
    
    //@ToDo: reset not mandatory
    image_rgb_.reset( new ImageMeasurement( image, tf, *camera_info_rgb_ ));
    
    if ( image_rgb_ && image_depth_ ) {
      ROS_INFO( "processImage" );
      img_processor_->processImage( image_rgb_, image_depth_ );
      image_rgb_ = nullptr;
      image_depth_ = nullptr;
    }
  }
}

void DoorDetectorNode::callbackCameraInfoRGB( const sensor_msgs::CameraInfoConstPtr &_msg ) {
  if ( !camera_info_rgb_ ) {
    ROS_INFO( "CameraInfo for RGB cam retrieved" );
  }
  camera_info_rgb_.reset( new sensor_msgs::CameraInfo( *_msg ));
}

void DoorDetectorNode::callbackCameraInfoDepth( const sensor_msgs::CameraInfoConstPtr &_msg ) {
  if ( !camera_info_depth_ ) {
    ROS_INFO( "CameraInfo for depth cam retrieved" );
  }
  camera_info_depth_.reset( new sensor_msgs::CameraInfo( *_msg ));
}

void DoorDetectorNode::callbackDepthImage( const sensor_msgs::ImageConstPtr &_img ) {
  
  if ( !camera_info_depth_ ) {
    return;
  }
  
  auto image = cv_bridge::toCvCopy( _img, std::string( "16UC1" ));
  image->image.convertTo( image->image, CV_32FC1,
                          1.0 / static_cast<double>(std::numeric_limits<u_int16_t>::max()));
  
  tf::StampedTransform tf;
  if ( getStaticTF( params_.world_frame, _img->header.frame_id.c_str(), tf, params_.debug )) {
    
    //@ToDo: reset not mandatory only image has changed
    image_depth_.reset( new ImageMeasurement( image, tf, *camera_info_depth_ ));
    
    if ( image_rgb_ && image_depth_ ) {
      img_processor_->processImage( image_rgb_, image_depth_ );
      image_rgb_ = nullptr;
      image_depth_ = nullptr;
    }
  }
}

void DoorDetectorNode::publish() {
  door_detector_->publish();
  img_processor_->display();
}

void DoorDetectorNode::callbackLaser( const sensor_msgs::LaserScan &_laser ) {
  //door_detector_->processLaser(_laser);
  
  std::unique_ptr<Contour> contour_vis;
  contour_vis.reset( new Contour());
  
  tf::StampedTransform tf;
  if ( getStaticTF( params_.world_frame, params_.laser_source_frame, tf, params_.debug )) {
    size_t n = _laser.ranges.size();
    laser_measurement_.reset( new LaserMeasurement( _laser, tf ));
    
    for ( int i = 0; i < n; ++i ) {
      double range = _laser.ranges[i];
      if ( isfinite( range ) && range < _laser.range_max ) {
        const double angle = _laser.angle_min + (_laser.angle_increment * i);
        const Point2D pt( cos( angle ) * range, sin( angle ) * range );
        laser_measurement_->push_back( Contour::Beam( range, angle, pt ));
        contour_vis->push_back( Contour::Beam::make_beam( range, angle, pt ));
      }
    }
    
    img_processor_->registerLaser( laser_measurement_ );
  }
  
}

bool DoorDetectorNode::getStaticTF( const std::string &world_frame, const std::string &source_frame,
                                    tf::StampedTransform &_pose, bool debug ) {
  
  std::string target_frame_id = source_frame;
  std::string source_frame_id = tf::resolve( "", world_frame );
  std::string key = target_frame_id + "->" + source_frame_id;
  
  if ( !tfMap_[key] ) {
    try {
      
      listenerTF_.lookupTransform(
          source_frame_id, target_frame_id, ros::Time( 0 ), _pose );
      std::shared_ptr<tf::StampedTransform> tf_ref;
      tf_ref.reset( new tf::StampedTransform( _pose ));
      tfMap_[key] = std::move( tf_ref );
      
    } catch (tf::TransformException ex) {
      
      ROS_INFO( "getStaticTF" );
      ROS_ERROR( "%s", ex.what());
      ros::Duration( 1.0 ).sleep();
      return false;
      
    }
  }
  
  _pose = *tfMap_[key];
  
  if ( debug ) {
    std::cout << key << std::endl;
    std::cout << "tf get o: " << _pose.getOrigin().x() << ", " << _pose.getOrigin().y() << ", "
              << _pose.getOrigin().z()
              << std::endl;
    std::cout << "tf get r: " << _pose.getRotation().x() << ", " << _pose.getRotation().y() << ", "
              << _pose.getRotation().z() << ", " << _pose.getRotation().w() << std::endl;
  }
  
  return true;
}


int main( int argc, char **argv ) {
  ros::init( argc, argv, "door_2d_detector_node" );
  
  DoorDetectorNode detector_node;
  
  while ( ros::ok()) {
    ros::spinOnce();
    detector_node.publish();
    
  }
  return 0;
}
