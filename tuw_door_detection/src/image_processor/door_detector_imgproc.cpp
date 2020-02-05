#include <imgproc/door_detector_imgproc.h>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <tuw_geometry/utils.h>

//
// Created by felix on 09.11.18.
//

using namespace tuw;
using namespace image_processor;

DoorDetectorImageProcessor::DoorDetectorImageProcessor() : last_door_detection_( std::make_shared<DoorDetection>())
{
}

DoorDetectorImageProcessor::~DoorDetectorImageProcessor() = default;

void DoorDetectorImageProcessor::processImage( std::shared_ptr<ImageMeasurement> &_image_meas_rgb,
                                               std::shared_ptr<ImageMeasurement> &_image_meas_depth )
{
  
  last_img_processed_ = _image_meas_rgb;
  last_depth_processed_ = _image_meas_depth;
  
  cv::Mat &img_cv = _image_meas_rgb->cv();
  cv::GaussianBlur( img_cv, img_cv, cv::Size( 5, 5 ), 0.75 );
  cv::Canny( img_cv, img_cv, 20, 55, 3, true );

  cv::Mat dbg_img;
  cv::cvtColor(img_cv, dbg_img, CV_GRAY2BGR);
  cv::Mat rtheta_img = cv::Mat::zeros(2*img_cv.rows,2*img_cv.cols,CV_8U);

  std::vector<cv::Vec2f> lines;
  //cv::HoughLinesP(img_cv, lines, 1, CV_PI/180, 50, 50, 10 );
  //for( size_t i = 0; i < lines.size(); i++ )
  //{
  //    cv::Vec4i l = lines[i];
  //    line( dbg_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
  //}
  //std::string img_name = "detected lines: ";
  //cv::imshow(img_name, dbg_img);

  //std::cout << "lines detected: " << lines.size();

  cv::HoughLines( img_cv, lines, 1, CV_PI/180, 100 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
      float rho = lines[i][0];
      float theta = lines[i][1];
      std::cout << "(r,theta) " << rho << " " << theta << std::endl;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      cv::Point pt1(cvRound(x0 + 1000*(-b)),
                cvRound(y0 + 1000*(a)));
      cv::Point pt2(cvRound(x0 - 1000*(-b)),
                cvRound(y0 - 1000*(a)));
      cv::line( dbg_img, pt1, pt2, cv::Scalar(0,0,255), 3, 8 );
  }

  cv::imshow("Lines", dbg_img);

  std::cout << "lines detected: " << lines.size();
  last_door_detection_->setImageMeasurement( _image_meas_rgb );
}

std::shared_ptr<DoorDetection> &DoorDetectorImageProcessor::getResult()
{
  return last_door_detection_;
}
