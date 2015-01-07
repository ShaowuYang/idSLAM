 /*
 * PTAMNode.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: scherer
 */

#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
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

class MonoPTAMNode : public BasePTAMNode {
public:
    MonoPTAMNode() :
        BasePTAMNode()
    {
        image_transport::ImageTransport it(nh_);
        image_sub_ = it.subscribe("image", 1, boost::bind(&MonoPTAMNode::imageCallback, this, _1));
        ImageRef size(640,480);
        frameRGB_.resize(size);
        frameBW_.resize(size);
    }

    ~MonoPTAMNode()
    {
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
        frame_stamp_ = img_msg->header.stamp.toSec();

        convertImage(img_msg);

        // pass RGBD frame along to PTAM
        tracker_->TrackFrame(frameBW_);
        cout << tracker_->GetMessageForUser() << endl;

        // publish current pose in map
        publishPose();

        if (cam_marker_pub_.getNumSubscribers() > 0 || point_marker_pub_.getNumSubscribers() > 0)
            map_viz_->publishMapVisualization(map_,tracker_,cam_marker_pub_,point_marker_pub_);

        if (show_debug_image_) {
            cv::Mat rgb_cv(480, 640, CV_8UC3, frameRGB_.data());
            map_viz_->renderDebugImage(rgb_cv,tracker_,map_);
            cv::imshow("debug",rgb_cv);
            cv::waitKey(10);
        }
    }

private:
    void convertImage(const sensor_msgs::ImageConstPtr& img_msg) {
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
    }

    image_transport::Subscriber image_sub_;

    CVD::Image<CVD::Rgb<CVD::byte> > frameRGB_;
    CVD::Image<CVD::byte> frameBW_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_ptam_node");

    MonoPTAMNode node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

