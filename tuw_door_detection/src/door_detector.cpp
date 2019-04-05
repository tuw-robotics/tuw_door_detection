//
// Created by felix on 14.12.18.
//

#include <door_detector.h>
#include <memory>
#include <opencv2/highgui.hpp>

using namespace tuw;

DoorDetector::DoorDetector() : pcl_doors_( new pcl::PointCloud<pcl::PointXYZ>()),
                               pcl_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.5f))
{
  pcl_octree_->setInputCloud(pcl_doors_);
  pcl_octree_->addPointsFromInputCloud();
}

DoorDetector::~DoorDetector()
{

}

void DoorDetector::lookupHistory( const Eigen::Vector3d &pose_ws )
{
  
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = 0.5;
  
  pcl::PointXYZ searchPoint( pose_ws.x(), pose_ws.y(), pose_ws.z());
  if ( pcl_octree_->radiusSearch( searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
  {
    for ( size_t i = 0; i < pointIdxRadiusSearch.size(); ++i )
      std::cout << "    " << pcl_doors_->points[pointIdxRadiusSearch[i]].x
                << " " << pcl_doors_->points[pointIdxRadiusSearch[i]].y
                << " " << pcl_doors_->points[pointIdxRadiusSearch[i]].z
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  } else
  {
    //octree.addPointToCloud(pose_ws, pcl_doors_);
    std::cout << "adding point to pointcloud" << std::endl;
    pcl_octree_->addPointToCloud( searchPoint, pcl_doors_ );
  }
}

bool DoorDetector::merge( std::shared_ptr<image_processor::DoorDetectorImageProcessor> &img_processor,
                          std::shared_ptr<door_laser_proc::DoorDetectorBase> &laser_processor )
{
  if ( !image_measurement_ || !laser_measurement_ )
  {
    return false;
  }
  
  auto laser_processor_dyn = std::dynamic_pointer_cast<door_laser_proc::DoorDepthDetector>(
      laser_processor );
  
  if ( !laser_processor_dyn )
  {
    return false;
  }
  
  detection_image_ = img_processor->getResult();
  detection_laser_ = laser_processor_dyn->getContours();
  
  const auto T_WC = image_measurement_->getTfWorldSensor();
  const auto T_WL = laser_measurement_->getTfWorldSensor();
  const auto z_laser = T_WL( 2, 3 );
  const Eigen::Matrix4d T_CL = T_WC.inverse() * T_WL;
  
  auto &cmodel = image_measurement_->getCameraModel();
  auto img_height = image_measurement_->cv().size().height;
  auto img_width = image_measurement_->cv().size().width;
  
  //bottom left, top left, top right, bottom right
  std::vector<cv::Point2d> bb;
  std::for_each( detection_laser_.begin(), detection_laser_.end(),
                 [&cmodel, &T_CL, img_height, img_width, z_laser, this]
                     ( std::shared_ptr<Contour> &contr )
                 {
                   //TODO clean up this mess
                   contr->registerToImage( T_CL, z_laser,
                                           cmodel->fx(), cmodel->fy(),
                                           cmodel->cx(), cmodel->cy(),
                                           cmodel->Tx(), cmodel->Ty());
                   //for (auto beam : contr->beams())
                   //{
                   //  std::cout << beam->img_coords << std::endl;
                   //}
                   //contr->registerFloorImage(T_WL)
                   contr->visibilityCheck( false, img_width, img_height );
                 } );
  
  //for ( const std::shared_ptr<Contour> &c : detection_laser_ )
  //{
  //  if ( c->is_door_candidate())
  //  {
  //    door_candidates_.push_back( c );
  //  }
  //  for ( auto &child_candidate : c->getChildren())
  //  {
  //    if ( child_candidate->is_door_candidate())
  //    {
  //      door_candidates_.push_back( child_candidate );
  //    }
  //  }
  //}
  if ( tf_baselink_world )
  {
    Eigen::Matrix4d tf_world_obj = tf_baselink_world->inverse() * T_WL;
    for ( std::shared_ptr<Contour> contr : detection_laser_ )
    {
      Eigen::Vector3d door_bot0 = contr->getBoundingBoxObjSpace().back();
      Eigen::Vector3d door_bot1 = contr->getBoundingBoxObjSpace().front();
      Eigen::Vector3d dir = door_bot1 - door_bot0;
      double length = dir.norm();
      dir.normalize();
      Eigen::Vector3d door_center_bot = (dir * (length / 2.0)) + door_bot0;
      Eigen::Vector4d dcb_ws = Eigen::Vector4d( door_center_bot.x(), door_center_bot.y(), door_center_bot.z(), 1 );
      dcb_ws = tf_world_obj * dcb_ws;
      dcb_ws = dcb_ws / dcb_ws[3];
      lookupHistory( dcb_ws.head<3>() );
    }
    //reset for the next method invocation (prevent multiple uses of same location at different times)
    tf_baselink_world = nullptr;
  }
  
  return true;
}

void DoorDetector::clear()
{
  image_measurement_ = nullptr;
  laser_measurement_ = nullptr;
  doors_.clear();
}

void DoorDetector::setImageMeasurement( std::shared_ptr<ImageMeasurement> &img_meas )
{
  image_measurement_ = img_meas;
}

void DoorDetector::setLaserMeasurement( std::shared_ptr<tuw::LaserMeasurement> &laser_meas )
{
  laser_measurement_ = laser_meas;
}

tuw_object_msgs::ObjectWithCovariance DoorDetector::generateObjMessage( std::shared_ptr<Contour> &contour, int32_t id )
{
  using tuw_object_msgs::ObjectWithCovariance;
  using tuw_object_msgs::Object;
  
  tuw_object_msgs::ObjectWithCovariance obj;
  obj.object.ids = {id};
  
  //@ToDo: which orientation how did i define it?
  //@ToDo: refine opening angle and bounding box
  Eigen::Vector3d pos;
  if ( contour->doorPostLeft())
  {
    pos = contour->getBoundingBoxObjSpace().back();
  } else
  {
    pos = contour->getBoundingBoxObjSpace().front();
  }
  
  obj.object.pose.position.x = pos.x();
  obj.object.pose.position.y = pos.y();
  obj.object.pose.position.z = pos.z();
  
  obj.object.pose.orientation.x = 0;
  obj.object.pose.orientation.y = 0;
  obj.object.pose.orientation.z = 0;
  obj.object.pose.orientation.w = 1;
  
  obj.object.shape = Object::SHAPE_DOOR;
  
  //@TODO magic values
  obj.object.shape_variables = {
      0.9,
      2.0,
      0,
      0,
      1,
      0
  };
  
  return std::move( obj );
}

void DoorDetector::setRobotPosition( Eigen::Matrix4d &tf_baselink_world )
{
  this->tf_baselink_world = std::make_shared<Eigen::Matrix4d>( tf_baselink_world );
}

tuw_object_msgs::ObjectDetection DoorDetector::getResultAsMessage()
{
  tuw_object_msgs::ObjectDetection det_msg;
  det_msg.header.stamp = ros::Time::now();
  det_msg.header.frame_id = laser_measurement_->getLaser().header.frame_id;
  det_msg.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_DOOR;
  
  int32_t id = 0;
  for ( std::vector<std::shared_ptr<Contour>>::iterator it_contour = detection_laser_.begin();
        it_contour < detection_laser_.end();
        ++it_contour )
  {
    std::shared_ptr<Contour> contour = *it_contour;
    if ( contour->is_door_candidate())
    {
      doors_.push_back( contour );
      det_msg.objects.push_back( std::move( generateObjMessage( contour, id++ )));
    }
    for ( auto chld: contour->getChildren())
    {
      if ( chld->is_door_candidate())
      {
        doors_.push_back( chld );
        det_msg.objects.push_back( std::move( generateObjMessage( chld, id++ )));
      }
    }
  }
  
  return std::move( det_msg );
}

void DoorDetector::display()
{
  
  if ( image_measurement_ && laser_measurement_ )
  {
    cv::Mat img_display;
    detection_image_->getImageMeasurement()->cv().copyTo( img_display );
    if ( img_display.channels() != 3 )
    {
      cv::cvtColor( img_display, img_display, CV_GRAY2BGR );
    }
    
    for ( const auto &contour : detection_laser_ )
    {
      
      for ( auto it_beams = contour->begin();
            it_beams != contour->end(); ++it_beams )
      {
        std::shared_ptr<Beam> &beam = *it_beams;
        
        double rad = 1;
        if ( it_beams == contour->begin() || it_beams == (contour->end() - 1))
        {
          rad = 2;
        }
        
        if ( beam->get_is_visible())
        {
          auto &img_pnt = beam->img_coords;
          cv::circle( img_display, img_pnt.cv(),
                      rad, contour->getAssignedColor(), rad );
          //auto &floor_pnt = beam->img_base_coords;
          //cv::circle(img_display, floor_pnt.cv(),
          //           rad, contour->getAssignedColor(), rad);
        }
      }
      
      for ( const auto &line_seg : contour->getLineSegmentImageCoords())
      {
        cv::Point2d p0 = line_seg.first.cv();
        cv::Point2d p1 = line_seg.second.cv();
        cv::line( img_display, p0, p1, cv::Scalar( 255, 255, 255 ), 2 );
      }
    }
    
    //std::cout << "door candidates size " << door_candidates_.size() << std::endl << std::endl;
    //
    for ( auto it_contour = detection_laser_.begin();
          it_contour != detection_laser_.end();
          ++it_contour )
    {
      std::shared_ptr<Contour> contour = *it_contour;
      if ( contour->is_door_candidate())
      {
        draw_roi( contour, img_display );
      }
      for ( auto chld: contour->getChildren())
      {
        if ( chld->is_door_candidate())
        {
          draw_roi( chld, img_display );
        }
      }
    }
    
    cv::namedWindow( "rgb image processed" );
    cv::imshow( "rgb image processed", img_display );
    cv::waitKey( 100 );
    
  }
  
  for ( const auto c : detection_laser_ )
  {
    //auto corners = c->getCorners();
    //std::cout << "c " << corners.size() << std::endl;
    //for ( const std::unique_ptr<Contour::Corner> &cn : corners )
    //{
    //}
  }
}

void
DoorDetector::draw_roi( std::shared_ptr<Contour> &contour, cv::Mat &img_display )
{
  std::vector<Point2D> points = contour->getBoundingBox();
  std::vector<cv::Point2i> points_cv( points.size());
  
  std::size_t i = 0;
  std::for_each( points.begin(), points.end(),
                 [&i, &points_cv]( const Point2D &e )
                 {
                   points_cv[i++] = std::move( cv::Point2i( e.x(), e.y()));
                 } );
  
  cv::polylines( img_display, points_cv, true,
                 cv::Scalar( 0, static_cast<int>(255 * contour->candidateLikelyhood()), 1, 8 ));
  
  if ( contour->doorPostLeft())
  {
    cv::line( img_display, points_cv[points_cv.size() - 1], points_cv[points_cv.size() - 2], cv::Scalar( 0, 0, 255 ));
  } else
  {
    cv::line( img_display, points_cv[0], points_cv[1], cv::Scalar( 0, 0, 255 ));
  }
}
