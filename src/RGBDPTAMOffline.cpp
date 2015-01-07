#include <cstdlib>

#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <cvd/draw.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/image_convert.h>
#include <gvars3/instances.h>

#include <highgui.h>

#include "conversions/conversions.h"

#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/MapPoint.h"
#include "ptam/Tracker.h"
#include "ptam/TrackerData.h"

#include "MapVisualization.h"


//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;


/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const> &msg)
    {
        signalMessage(msg);
    }
};

class RGBDPTAMOffline
{
public:
    RGBDPTAMOffline()
    {
        cam_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ptam/cameras", 1);
        point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ptam/points", 1);
        cam_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ptam/pose",1);

        CVD::ImageRef size(640,480);
        frameD_.resize(size);
        frameRGB_.resize(size);
        frameBW_.resize(size);

        GVars3::GUI.LoadFile("settings.cfg");

        map_ = new Map;
        mapmaker_ = new MapMaker(*map_, true); // offline mode
        tracker_ = new Tracker(*map_, *mapmaker_);
        mapviz_ = new MapVisualization();

        pos_stream_.open("poses.csv");
    }

    // Load bag
    void loadBag(const std::string &filename)
    {
        rosbag::Bag bag;
        bag.open(filename, rosbag::bagmode::Read);

        std::string rgb_topic_name =   "/camera/rgb/image_color";
        std::string depth_topic_name = "/camera/depth/image_raw";

        // Image topics to load
        std::vector<std::string> topics;
        topics.push_back(rgb_topic_name);
        topics.push_back(depth_topic_name);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // Set up fake subscribers to capture images
        BagSubscriber<sensor_msgs::Image> rgb_sub, depth_sub;

        // Use time synchronizer to make sure we get properly synchronized images
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), rgb_sub, depth_sub);
        sync.registerCallback(boost::bind(&RGBDPTAMOffline::callback, this, _1, _2));

        // Load all messages into our stereo dataset
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
//            std::cout << "got mesage on topic: " << m.getTopic() << std::endl;
            if (m.getTopic() == rgb_topic_name || ("/" + m.getTopic() == rgb_topic_name))
            {
                sensor_msgs::Image::ConstPtr rgb_image = m.instantiate<sensor_msgs::Image>();
                if (rgb_image != NULL) {
                    rgb_sub.newMessage(rgb_image);
                }
            }

            if (m.getTopic() == depth_topic_name || ("/" + m.getTopic() == depth_topic_name))
            {
                sensor_msgs::Image::ConstPtr depth_image = m.instantiate<sensor_msgs::Image>();
                if (depth_image != NULL) {
                    depth_sub.newMessage(depth_image);
                }
            }

        }
        bag.close();
    }

    // Callback for synchronized messages
    void callback(const sensor_msgs::ImageConstPtr& img_msg,
                  const sensor_msgs::ImageConstPtr& depth_img_msg)
    {
        // convert to CVD image
        CVD::ImageRef img_size(img_msg->width,img_msg->height);
        if (img_msg->encoding.compare("rgb8") == 0) {
            memcpy(frameRGB_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
            CVD::convert_image(frameRGB_,frameBW_);
        } else if (img_msg->encoding.compare("mono8") == 0) {
            memcpy(frameBW_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
            CVD::convert_image(frameBW_,frameRGB_);
        }
        memcpy(frameD_.data(),&(depth_img_msg->data[0]),depth_img_msg->step*img_size.y);

        // track frame
        tracker_->TrackFrame(frameBW_, frameD_);
        // if new keyframe: run mapmaker
        if (mapmaker_->QueueSize() > 0) {
            mapmaker_->OfflineStep(false);
        }

        std::string message = tracker_->GetMessageForUser();
        std::cout << message << std::endl;


        // publish map
        if (cam_marker_pub_.getNumSubscribers() > 0 || point_marker_pub_.getNumSubscribers() > 0)
            mapviz_->publishMapVisualization(map_,tracker_,cam_marker_pub_,point_marker_pub_);

        SE3<> camPose = tracker_->GetCurrentPose().inverse();
        geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped);
        pose->header.frame_id = "/map";
        pose->pose.position.x = camPose.get_translation()[0];
        pose->pose.position.y = camPose.get_translation()[1];
        pose->pose.position.z = camPose.get_translation()[2];
        SO3<> rotation = camPose.get_rotation();
        const Matrix<3,3> Rc = rotation.get_matrix();
        quat_from_so3(pose->pose.orientation, rotation);
        cam_pose_pub_.publish(pose);

        Matrix<3,3> camToWorld;
        camToWorld[0][0] =  0; camToWorld[0][1] =  0; camToWorld[0][2] = 1;
        camToWorld[1][0] = -1; camToWorld[1][1] =  0; camToWorld[1][2] = 0;
        camToWorld[2][0] =  0; camToWorld[2][1] = -1; camToWorld[2][2] = 0;

        Matrix<3,3> camToWorldT;
        camToWorldT[0][0] =  0; camToWorldT[0][1] =  -1; camToWorldT[0][2] = 0;
        camToWorldT[1][0] =  0; camToWorldT[1][1] =  0; camToWorldT[1][2] = -1;
        camToWorldT[2][0] =  1; camToWorldT[2][1] =  0; camToWorldT[2][2] = 0;


        Matrix<3,3> R = camToWorld*Rc*camToWorldT;

        double pitch = atan2(-R[2][0],sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]));
        double cpitch = cos(pitch);
        double yaw, roll;
        /*if (fabs(pitch - M_PI_2) < 1e-4) {
            yaw = 0;
            roll = atan2(R[0][1],R[1][1]);
        } else if (fabs(pitch + M_PI_2) < 1e-4) {
            yaw = 0;
            roll = -atan2(R[0][1],R[1][1]);
        } else */{
            yaw = atan2(R[1][0]/cpitch,R[0][0]/cpitch);
            roll = atan2(R[2][1]/cpitch,R[2][2]/cpitch);
        }

        pos_stream_.precision(20);
        pos_stream_ << img_msg->header.stamp.toSec() << " "
                    << pose->pose.position.z <<  " "
                    << -pose->pose.position.x <<  " "
                    << -pose->pose.position.y << " "
                    << roll << " "
                    << pitch << " "
                    << yaw << std::endl;

        if (tracker_->GetNFrame() == tracker_->GetNLastKeyFrame()
                                 || tracker_->GetNFrame() == 1) {
            static GVars3::gvar3<int> gvnWriteFrame("MapMaker.WriteFrames",0,GVars3::SILENT);
            mapviz_->writeFrame(img_msg, depth_img_msg, tracker_->GetNKeyFrames());
        }


        if (true) {
            cv::Mat rgb_cv(480, 640, CV_8UC3, frameRGB_.data());
            mapviz_->renderDebugImage(rgb_cv,tracker_,map_);
            cv::imshow("debug",rgb_cv);
            cv::waitKey(10);
        }
    }

    void finishMap() {
//        for (int i = 0; i < 10; i++) {
//            mapmaker_->OfflineStep(true);
//        }
    }

    void dumpMap(const char* filename) {
        mapmaker_->WriteFrames("frames.csv");

        // count good points
        int nGoodPoints = 0;
        for (unsigned int i = 0; i < map_->vpPoints.size(); i++) {
            if  (!map_->vpPoints[i]->bBad && map_->vpPoints[i]->nMEstimatorInlierCount >= 3)
                nGoodPoints++;
        }

        FILE* out = fopen(filename,"w");
        fprintf(out,"ply\n");
        fprintf(out,"format ascii 1.0\n");
        fprintf(out,"element vertex %i\n",nGoodPoints);
        fprintf(out,"property float x\n");
        fprintf(out,"property float y\n");
        fprintf(out,"property float z\n");
        fprintf(out,"property uchar red\n");
        fprintf(out,"property uchar green\n");
        fprintf(out,"property uchar blue\n");
        fprintf(out,"end_header\n");

        for (unsigned int i = 0; i < map_->vpPoints.size(); i++) {
            if (!map_->vpPoints[i]->bBad && map_->vpPoints[i]->nMEstimatorInlierCount >= 3) {
            	boost::shared_ptr<KeyFrame> kf = map_->vpPoints[i]->pPatchSourceKF.lock();
            	if(kf.get() == NULL)
            		continue;
            
                int gray = kf->aLevels[map_->vpPoints[i]->nSourceLevel].im[map_->vpPoints[i]->irCenter];
                fprintf(out,"%f %f %f %u %u %u\n",
                        map_->vpPoints[i]->v3WorldPos[0],
                        map_->vpPoints[i]->v3WorldPos[1],
                        map_->vpPoints[i]->v3WorldPos[2],
                        gray, gray, gray);
            }

        }
        fclose(out);
    }

    CVD::Image<CVD::Rgb<CVD::byte> > frameRGB_;
    CVD::Image<CVD::byte> frameBW_;
    CVD::Image<uint16_t> frameD_;

    Map *map_;
    MapMaker *mapmaker_;
    Tracker *tracker_;
    MapVisualization* mapviz_;

    ros::NodeHandle nh_;
    ros::Publisher cam_marker_pub_;
    ros::Publisher point_marker_pub_;
    ros::Publisher cam_pose_pub_;

    // logging the trajectory
    std::ofstream pos_stream_;
};


int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "usage: ptamoffline bagfile" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string filename = argv[1];

    ros::init(argc, argv, "ptam_offline");
    RGBDPTAMOffline ptam;

    std::cout << "loading bag: " << filename << " ... " << std::endl;
    ptam.loadBag(filename.c_str());

    // once done:
    std::cout << "finishing map... this may a while" << std::endl;
    ptam.finishMap();

    std::cout << "dumping map" << std::endl;
    ptam.dumpMap("map.ply");
}
