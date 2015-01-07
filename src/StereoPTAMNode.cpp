#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ptam/MultiCameraImage.h>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include "ptam/Tracker.h"
#include "BasePTAMNode.h"
#include "MapVisualization.h"

using namespace CVD;
using namespace std;
using namespace cv;

// ROS Node for parallel tracking and mapping (PTAM) with
// sparse stereo data
class StereoPTAMNode : public BasePTAMNode {
public:
	StereoPTAMNode()
		:BasePTAMNode(), multiCamSubscriber(nh_, "capture", 1), pointsSubscriber(nh_, "sparse_stereo", 1),
		synchronizer(CameraPointsSyncPolicy(2), multiCamSubscriber, pointsSubscriber), lastLog(0) {
		synchronizer.registerCallback(bind(&StereoPTAMNode::synchronizerCallback, this, _1, _2));
	}

private:
	typedef message_filters::sync_policies::ExactTime<ptam::MultiCameraImage, sensor_msgs::PointCloud> CameraPointsSyncPolicy;

	// ROS subscriptions
	message_filters::Subscriber<ptam::MultiCameraImage> multiCamSubscriber;
	message_filters::Subscriber<sensor_msgs::PointCloud> pointsSubscriber;
	message_filters::Synchronizer<CameraPointsSyncPolicy> synchronizer;
	
	time_t lastLog;
	
	void synchronizerCallback(const ptam::MultiCameraImageConstPtr& multiCam, const sensor_msgs::PointCloudConstPtr& points) {
		// Convert cam0 to opencv image, no data is copied
		Mat_<unsigned char> cam0Img = cv_bridge::toCvShare(multiCam->cam0, multiCam)->image;
		// Convert to cvd sub-image, no data is copied
		CVD::SubImage<CVD::byte> cvdSubImg(cam0Img.data, ImageRef(cam0Img.cols, cam0Img.rows), cam0Img.step[0]);
		// Convert to cvd image, pixel data is copied
		CVD::Image<CVD::byte> cvdImg(ImageRef(cam0Img.cols, cam0Img.rows));
		cvdImg.copy_from(cvdSubImg);
	
		tracker_->TrackFrame(cvdImg, *points);
		if(lastLog != time(NULL)) {
			cout << tracker_->GetMessageForUser() << endl;
			lastLog = time(NULL);
		}
		
		// Publish messages
		publishPose(multiCam->header.stamp);
		if (cam_marker_pub_.getNumSubscribers() > 0 || point_marker_pub_.getNumSubscribers() > 0)
			map_viz_->publishMapVisualization(map_,tracker_,cam_marker_pub_,point_marker_pub_);
		
		// Display debugging image
		if (show_debug_image_) {
			Mat_<Vec3b> debImg(cam0Img.size());
			cvtColor(cam0Img, debImg, CV_GRAY2RGB);
			map_viz_->renderDebugImage(debImg, tracker_, map_);
			imshow("debug",debImg);
			waitKey(10);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_ptam_node");

	StereoPTAMNode node;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}
