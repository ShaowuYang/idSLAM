 /*
 * PTAMNode.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: scherer
 */

#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <highgui.h>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/image_convert.h>

#include "ptam/Tracker.h"

#include "BasePTAMNode.h"
#include "MapVisualization.h"

using namespace CVD;
using namespace std;

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;

class RGBDPTAMNode : public BasePTAMNode {
public:
    RGBDPTAMNode() :
        BasePTAMNode(),
        sub_rgb_(nh_, "rgb", 1),
        sub_depth_(nh_, "depth", 1),
        sync_(MySyncPolicy(2),  sub_rgb_, sub_depth_)
    {
        MySyncPolicy* policy = static_cast<MySyncPolicy*>(sync_.getPolicy());
        policy->setMaxIntervalDuration(ros::Duration(0.02));
        sync_.registerCallback(boost::bind(&RGBDPTAMNode::rgbdCallback, this, _1, _2));

        ImageRef size(640,480);
        frameD_.resize(size);
        frameRGB_.resize(size);
        frameBW_.resize(size);
    }

    ~RGBDPTAMNode()
    {
    }


    void rgbdCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_img_msg) {
        frame_stamp_ = img_msg->header.stamp.toSec();

        convertImages(img_msg,depth_img_msg);

        // pass RGBD frame along to PTAM
        tracker_->TrackFrame(frameBW_, frameD_);
        cout << tracker_->GetMessageForUser() << endl;

        // publish current pose in map
        publishPose();

        if (cam_marker_pub_.getNumSubscribers() > 0 || point_marker_pub_.getNumSubscribers() > 0)
            map_viz_->publishMapVisualization(map_,tracker_,cam_marker_pub_,point_marker_pub_);

        if (show_debug_image_) {
            cv::Mat rgb_cv(480, 640, CV_8UC3, frameRGB_.data());
            cv::cvtColor(rgb_cv,rgb_cv,CV_RGB2BGR);
            map_viz_->renderDebugImage(rgb_cv,tracker_,map_);
            cv::imshow("debug",rgb_cv);
            cv::waitKey(10);
        }

        if (write_keyframes_ && (tracker_->GetNFrame() == tracker_->GetNLastKeyFrame()
                                 || tracker_->GetNFrame() == 1)) {
            map_viz_->writeFrame(img_msg, depth_img_msg, tracker_->GetNKeyFrames());
        }
    }

private:
    void convertImages(const sensor_msgs::ImageConstPtr& img_msg,
                       const sensor_msgs::ImageConstPtr& depth_img_msg) {
        // convert image message to CVD image
        ImageRef img_size(img_msg->width,img_msg->height);
        if (img_msg->encoding.compare("rgb8") == 0) {
            memcpy(frameRGB_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
            CVD::convert_image(frameRGB_,frameBW_);
        } else if (img_msg->encoding.compare("mono8") == 0) {
            memcpy(frameBW_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
            CVD::convert_image(frameBW_,frameRGB_);
        } else {
            std::cerr << "error: ignoring image of unknown encoding: " << img_msg->encoding << std::endl;
            return;
        }

        // convert depth image message to CVD image
        if (depth_img_msg->encoding.compare("16SC1") != 0 && depth_img_msg->encoding.compare("16UC1") != 0) {
            std::cerr << "error: ignoring depth image of unknown encoding: " << depth_img_msg->encoding << std::endl;
            return;
        }
        memcpy(frameD_.data(),&(depth_img_msg->data[0]),depth_img_msg->step*img_size.y);
    }


    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
    message_filters::Synchronizer<MySyncPolicy> sync_;

    CVD::Image<CVD::Rgb<CVD::byte> > frameRGB_;
    CVD::Image<CVD::byte> frameBW_;
    CVD::Image<uint16_t> frameD_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_ptam_node");

    RGBDPTAMNode node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

