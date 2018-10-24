#include "door_depth_detector.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <tuw_geometry/figure.h>
#include <tuw_geometry/point2d.h>

using namespace tuw;

DoorDepthDetector::ParametersNode::ParametersNode() : node("~")
{
	node.param<std::string>("source_frame", source_frame, "/base_link");
	node.param<std::string>("world_frame", world_frame, "/map");
	node.param<bool>("debug", debug, false);
	node.param<std::string>("publisher_topic", publisher_topic, "/door_detections");
	node.param<std::string>("internal_mode", internal_mode, "kernel");
}

DoorDepthDetector::DoorDepthDetector(ros::NodeHandle &_nh) : params_(new DoorDepthDetector::ParametersNode()), DoorDetector(_nh), figure_local_("KernelResponses")
{
	init(params_->publisher_topic);
	
	figure_local_.init(config_.map_pix_x, config_.map_pix_y,
                     config_.map_min_x, config_.map_max_x,
                     config_.map_min_y, config_.map_max_y,
                     config_.map_rotation + M_PI/2.0,
                     config_.map_grid_x, config_.map_grid_y );
}

DoorDepthDetector::~DoorDepthDetector()
{
}

void DoorDepthDetector::plot(const sensor_msgs::LaserScan &_scan, std::vector<float> &_responses)
{
  cv::Mat plotResponses;
	cv::Mat responseMat(_responses);
	responseMat.convertTo(responseMat, CV_64F);
	cv::Ptr<cv::plot::Plot2d> plot = cv::plot::createPlot2d(responseMat);
	plot->render(plotResponses);
	cv::imshow("plot", plotResponses);
	
	double min_resp = std::numeric_limits<double>::max();
	double max_resp = -std::numeric_limits<double>::max();
	
	std::cout << std::endl << std::endl;
	for (const float r : _responses)
	{
		std::cout << r << ", ";
		min_resp = std::min(fabs(static_cast<double>(r)), min_resp);
		max_resp = std::max(fabs(static_cast<double>(r)), max_resp);
	}
	std::cout << std::endl << "min: " << min_resp << std::endl;
	std::cout << "max: " << max_resp << std::endl;
	
	std::vector<float> responses(_responses);
	for (std::vector<float>::iterator it = responses.begin();
			 it != responses.end();
	     ++it)
	{
		*it = ((fabs(*it) - min_resp) / (max_resp - min_resp));
	}
	
  figure_local_.clear();
  size_t N = _scan.ranges.size();
  for ( size_t i = 0; i < N; i++ ) 
  {
      /**
      * @ToDo Wanderer
      * uses Figure::symbol or Figure::circle to plot the laser measurements in a for loop
      **/
      //figure_local_.symbol(measurement_laser_[i].end_point, Figure::red);
	  	if (isfinite(_scan.ranges[i]) && _scan.ranges[i] < _scan.range_max)
			{
	  		Eigen::Vector2d point_meas;
				point_meas = range2Eigen(_scan, i);
				cv::Scalar color;
				if (responses[i] > 0.2)
				{
					std::cout << responses[i] << ", "; 
					color = cv::Scalar(255.0 * responses[i], 0, 0);
				}
				else 
				{
					color = cv::Scalar(0,0,0);
				}
   	  	figure_local_.circle(tuw::Point2D(point_meas.x(), 
   	  	                             	    point_meas.y()), 
   	  	                      						3.0, 
	  	  	                 								color);
                           								
			}
		
	}
	
  cv::imshow ( figure_local_.title(),figure_local_.view() );
	cv::waitKey(1);
}

//Border mode only same supported
bool DoorDepthDetector::kernelMode(const sensor_msgs::LaserScan &_scan, std::vector<Eigen::Vector2d> &_detections)
{
	assert (KERNEL_SIZE % 2 == 0);
	
	std::vector<float> ranges = std::vector<float>(_scan.ranges);
	
	std::vector<float> kernel(KERNEL_SIZE);
	size_t half_size = static_cast<std::size_t>(KERNEL_SIZE / 2.0);
	
	//TODO: preallocate
	//Same border mode
	for (int i=0; i < KERNEL_SIZE; ++i)
	{
		if (i < half_size)
		{
			kernel[i] = -1.0f;
			//slow
			ranges.insert(ranges.begin(), ranges[0]);
		}
		else 
		{
			kernel[i] = 1.0f;
			ranges.push_back(ranges.back());
		}
	}
	size_t N = ranges.size() - half_size;
	std::vector<float> responses;
	
	for (int i=half_size; i < N; ++i)
	{
		float length = ranges[i];

    if ( ( length < _scan.range_max ) && isfinite ( length ) ) 
		{
	     float sum = 0.0;
	     size_t total_inner_loop = KERNEL_SIZE;
	     for (int j = 0; j < KERNEL_SIZE; ++j)
	     {
		     float curr_range = ranges[(i - half_size) + j];
		     //HOW TO DEAL WITH THIS APPROPRIATELY?
		     if (!isfinite(curr_range))
		     {
			     total_inner_loop--;
			     continue;
		     }               
		     sum += (kernel[j] * curr_range);
	     }
	     sum /= static_cast<float>(total_inner_loop);
	     if (!isfinite(sum))
	     {
		     sum = 0;
	     }
	     responses.push_back(sum);
		}
	}
	
	plot(_scan, responses);
	
}

bool DoorDepthDetector::structureMode(const sensor_msgs::LaserScan &_scan, std::vector<Eigen::Vector2d> &_detections)
{
	//Eigen::Matrix<double,4,4> tf_laser2world = tf2EigenMat(tf_laser); 
	//std::cout << "robot pose: (" << tf_laser2world(0,3) << ", " << tf_laser2world(1,3) << ")" << std::endl;
	
	size_t N = _scan.ranges.size();
	
	int i=0;
	float last_range = _scan.ranges[0];
	int last_range_idx = 0;
	while(!isfinite(last_range) && i < N)
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

    if ( ( length < _scan.range_max ) && isfinite ( length ) && abs_diff > thresh_ ) 
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
				_detections.push_back(candidates[i].first);
				idx_set.insert(c.first);
			}
			
			if (idx_set.count(c.second) == 0)
			{
				_detections.push_back(candidates[i].second);
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
	if (!getTF(params()->world_frame, params()->source_frame, tf_laser, params()->debug))
	{
		return false;
	}
	
	std::vector<Eigen::Vector2d> detections;
	if (params()->internal_mode == "structure")
	{
		structureMode(_scan, detections);
	}
	else 
	{
		kernelMode(_scan, detections);
	}
	
	//t = tf_laser2world * t;
	for (Eigen::Vector2d &v : detections)
	{	
			Eigen::Quaterniond q(1,0,0,0);
			tuw_object_msgs::ObjectWithCovariance out_obj;
			out_obj.object.pose.position.x = v[0];				
			out_obj.object.pose.position.y = v[1];				
			out_obj.object.pose.position.z = 0;//v[2];				
			
			out_obj.object.pose.orientation.x = q.x();
			out_obj.object.pose.orientation.y = q.y();
			out_obj.object.pose.orientation.z = q.z();
			out_obj.object.pose.orientation.w = q.w();
			
			out_obj.object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
			out_obj.object.ids = { static_cast<int>(objects_.size()) };
			out_obj.object.ids_confidence = {0};
			out_obj.object.shape_variables = {0.1, 1, 0.5}; 
			objects_.push_back(std::move(out_obj));
	}
	
	return true;
}


