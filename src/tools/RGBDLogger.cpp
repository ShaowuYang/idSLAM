/*
 * RGBDLogger.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: scherer
 */

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <highgui.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace cv;

namespace rgbdslam {

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;

class RGBDLogger {
public:
  RGBDLogger() :
    frameId_(0),
    sub_rgb_(nh_, "/camera/rgb/image_color", 1),
    sub_depth_(nh_, "/camera/depth/image_raw", 1),
    sync_(MySyncPolicy(20),  sub_rgb_, sub_depth_)
  {
    // Set maximum time difference between depth and rgb to 0.02s.

    MySyncPolicy* policy = static_cast<MySyncPolicy*>(sync_.getPolicy());
    policy->setMaxIntervalDuration(ros::Duration(0.03));
    namedWindow("rgb");
    namedWindow("depth");
    sync_.registerCallback(boost::bind(&RGBDLogger::cameraCallback, this, _1, _2));
  }

  ~RGBDLogger()
  {
  }

  void cameraCallback(const sensor_msgs::ImageConstPtr& rgb_img_msg,
                      const sensor_msgs::ImageConstPtr& depth_img_msg) {
      frameId_++;

      cv_bridge::CvImagePtr cv_ptr_rgb;
      cv_bridge::CvImagePtr cv_ptr_depth;
      try
      {
          cv_ptr_rgb = cv_bridge::toCvCopy(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
          if (depth_img_msg->encoding == "16SC1") {
              cv_ptr_depth = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16SC1);
          } else if (depth_img_msg->encoding == "16UC1") {
              cv_ptr_depth = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
          } else {
              std::cerr << __PRETTY_FUNCTION__ << "unknown depth encoding: " << depth_img_msg->encoding << std::endl;
          }

          imshow("rgb",cv_ptr_rgb->image);
          imshow("depth",cv_ptr_depth->image);
          int key = waitKey(20);

          if (key > 0) {
              std::cout << "Key pressed, saving frame." << std::endl;
              std::stringstream rgbname, depthname, pointsname;
              rgbname << rgb_img_msg->header.stamp << ".png";
              depthname << rgb_img_msg->header.stamp << ".yml";
              pointsname << rgb_img_msg->header.stamp << ".csv";
              imwrite(rgbname.str(),cv_ptr_rgb->image);

              FileStorage fs(depthname.str(), FileStorage::WRITE);
              fs << "depth" << cv_ptr_depth->image;
              fs.release();


              //          std::ofstream of(pointsname.str().c_str());
              //          cam_model_.fromCameraInfo(camera_info_msg);
              //          for (int y = 0; y < depth_img.rows; y++) {
              //              for (int x = 0; x < depth_img.cols; x++) {
              //                  cv::Point p2i(x,y);

              //                  double d = depth_img.at<float>(cv::Point(x,y));

              //                  if (!isnan(d)) {
              //                      cv::Point3d p3d = cam_model_.projectPixelTo3dRay(p2i)*d;
              //                      of << p3d.x << " " << p3d.y << " " << p3d.z << std::endl;
              //                  }
              //              }
              //          }
              //          of.close();
          } // if key pressed
      } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      } // end catch
  }
private:
  int frameId_;
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgb_info_;
  message_filters::Synchronizer<MySyncPolicy> sync_;

  image_geometry::PinholeCameraModel cam_model_;

};

} // namespace rgbdslam


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbdlogger");

  rgbdslam::RGBDLogger node;

  ros::spin();

  return 0;
}
