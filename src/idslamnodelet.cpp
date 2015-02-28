/*
*  idslamnodelet.cpp
*  indoor multi-RGBD-Camera SLAM
*  Created on: Oct 5, 2014
*  Author: Shaowu Yang
*  Email: shaowu.yang@nudt.edu.cn
*  SLAM with kinect cameras (two in our case) based on PTAM and depth constraints,
*  working with a back end to be a full SLAM system
*/

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <highgui.h>

#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/signals.hpp>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/image_convert.h>
#include <TooN/wls.h>
#include <tag/absorient.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "ptam/Tracker.h"
#include "BaseSLAMNode.h"
#include "MapVisualization.h"
//#include <pximu/AttitudeData.h>
//#include <pximu/RawIMUData.h>
#include "conversions/conversions.h"

#include <idSLAM/Edges.h>
#include <idSLAM/Keyframe.h>
#include <idSLAM/Level.h>
#include <idSLAM/Mappoint.h>

//#include <backend/cslam/slam/Keyframe.h>
//#include <backend/cslam/cs_geometry/Conversions.h>
//#include <sophus/sim3.hpp>
//#include <sophus/se3.hpp>
//#include "backend/cslam/backend.h"

#include "ptam/KeyFrame.h"
#include "ptam/Tracker.h"
#include "ptam/MapPoint.h"
#include "ptam/CameraModel.h"

using namespace CVD;
using namespace std;
using namespace backend;

namespace idSLAM{
#define intervalmax_imgdual 1/30.00    // time interval of img msg from different camera

class idslamnodelet : public BaseSLAMNode{
public:
    idslamnodelet() : BaseSLAMNode(){}

    virtual void onInit(){
        BaseSLAMNode::onInit();

        checkparam();
//        se3IMUfromcam = Load_cam_para();
        param_ini();

        image_transport::ImageTransport it(nh_);

        cam_posewithcov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ptam_world_cov",1);
        quad_pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(pose_,1);
        cam_marker_pub_sec = nh_.advertise<visualization_msgs::Marker>("/ptam/camerasec", 1);

        rgbSubscriber_   = it.subscribe("rgb", 1, boost::bind(&idslamnodelet::rgbCallback, this, _1));
        depthSubscriber_ = it.subscribe("depth", 1, boost::bind(&idslamnodelet::depthCallback, this, _1));
        rgbSubscriber1_   = it.subscribe("rgb1", 1, boost::bind(&idslamnodelet::rgbCallback1, this, _1));
        depthSubscriber1_ = it.subscribe("depth1", 1, boost::bind(&idslamnodelet::depthCallback1, this, _1));
        rgbSubscriber2_   = it.subscribe("rgb2", 1, boost::bind(&idslamnodelet::rgbCallback2, this, _1));
        depthSubscriber2_ = it.subscribe("depth2", 1, boost::bind(&idslamnodelet::depthCallback2, this, _1));

        if (isdualcam)
        {
            se3cam1fromcam2 = Load_camsec_para();
            tracker_->Load_Cam1FromCam2(se3cam1fromcam2);
        }
    }

    ~idslamnodelet()
    {
        if (pos_log_.is_open())
            pos_log_.close();
        if (debugloop.is_open())
            debugloop.close();
        if (timecost.is_open())
            timecost.close();
    }

    void checkparam()
    {
        if(!nh_private_.getParam("image", image_))
            image_ = "/camera/image_raw";
        if(!nh_private_.getParam("imagesec", image_sec))
            image_sec = "/forward/camera/image_raw";
        if(!nh_private_.getParam("pose", pose_))
            pose_ = "vicon_pose";
        if(!nh_private_.getParam("use_ekf", use_ekf_pose))
            use_ekf_pose = true;
        if(!nh_private_.getParam("sendvisual", sendvisual))
            sendvisual = false;

        if (!nh_private_.getParam("use_artificial_ini", use_circle_ini_))
            use_circle_ini_ = false;
        if (use_circle_ini_)
            tracker_->use_circle_ini = true;

        if (!nh_private_.getParam("cam_imu_file", cam_para_path))
            cam_para_path = "data/parameter_wrt.txt";
        if (!nh_private_.getParam("cam_sec_imu_file", cam_para_path_second))
            cam_para_path_second = "data/cameras_wrt.txt";

        if (!nh_private_.getParam("ref_img_path", ref_img_path))
            ref_img_path = "data/refpattern.jpg";

        if (!nh_private_.getParam("flying", isflying))
            isflying = false;
        if (isflying)
            tracker_->isflying = true;

        if (!nh_private_.getParam("trackingpad", istrackPad))
            istrackPad = false;
        if (istrackPad)
            tracker_->istrackPad = true;

        if (!nh_private_.getParam("ini_method", ini_method_))
            ini_method_ = "kinect";
        if (ini_method_ == "one_circle") {
            ini_one_circle_ = true;
            tracker_->use_one_circle = true;
            ini_ground_ = false;
            ini_two_circle_ = false;
        }
        else if (ini_method_ == "ground") {
            ini_ground_ = true;
            tracker_->use_ground = true;
            ini_one_circle_ = false;
            ini_two_circle_ = false;
        }
        else if (ini_method_ == "two_circle") {
            ini_two_circle_ = true;
            tracker_->use_two_circle = true;
            ini_one_circle_ = false;
            ini_ground_  =false;
        }
        else if (ini_method_ == "kinect") {
            ini_kinect_ = true;
            ini_ground_ = false;
            ini_two_circle_ = false;
            tracker_->use_kinect = true;
            ini_one_circle_ = false;
            ini_ground_  =false;
        }

        pos_log_.open("ptam.log");
        if (write_pos_)
            cout <<"logfile open?: " << pos_log_.is_open() << endl;
        if (pos_log_.is_open()){
            pos_log_.setf(std::ios::fixed, std::ios::floatfield);
            pos_log_.precision(10);
        }
        debugloop.open("debugloop.log");
        if (write_pos_)
            cout <<"loop logfile open?: " << debugloop.is_open() << endl;
        if (debugloop.is_open()){
            debugloop.setf(std::ios::fixed, std::ios::floatfield);
            debugloop.precision(10);
        }
        timecost.open("timecost.log");
        if (write_pos_)
            cout <<"loop logfile open?: " << timecost.is_open() << endl;
        if (timecost.is_open()){
            timecost.setf(std::ios::fixed, std::ios::floatfield);
            timecost.precision(10);
        }


        vis_publish_interval_ = 5.0;
        vis_pointcloud_step_ = 3;
        cellSize = 0.02;
    }

    void param_ini(){
        secondimgcame = false;
        thirdimgcame = false;
        secondepthcame = false;
        thirdepthcame = false;
        isviconcall = false;
        isimage_ini_call = false;
        ispose_predicted_ekfcall = false;
        isattitudecall = false;
        inipose_published = false;
        isEKFattitudeInicall = false;
        useEKFiniAtti = use_ekf_pose;
        isFinishIniPTAMwithcircle = false;
        isPTAMshouldstop = false;

        scale_ekf_ = 1;

        ImageRef size(640,480);
        frameRGB_.resize(size);
        frameBW_.resize(size);
        frameRGB_sec.resize(size);
        frameBW_sec.resize(size);
        frameRGB_third.resize(size);
        frameDepth_.resize(size);
        frameDepth_sec.resize(size);
        frameDepth_third.resize(size);
    }

    SE3<> Load_cam_para()
    {
        SE3<> campara;
        ifstream cam_para_table;
        string table_path = cam_para_path;
        double temp1, temp2, temp3, temp4, temp5;
        cam_para_table.open(table_path.c_str());
        assert(cam_para_table.is_open());

        cam_para_table>>temp1>>temp2>>temp3>>temp4>>temp5;
        Matrix<3> cam;
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            campara.get_translation()[i] = temp1/1000.0;
        }
        campara.get_rotation() = cam;

        return campara;//Tic
    }

    // the second camera pose in the master camera frames
    vector<SE3<> > Load_camsec_para()
    {
        vector<SE3<> > campara;
        ifstream cam_para_table;
        string table_path = cam_para_path_second;
        cout << "Camera relative pose file: " << table_path.c_str() << endl;
        double temp1;
        cam_para_table.open(table_path.c_str());
        assert(cam_para_table.is_open());

//        cam_para_table>>temp1>>temp2>>temp3>>temp4>>temp5;
        Matrix<3> cam;
        SE3<> campara_temp;
        for (int cn = 0; cn < AddCamNumber; cn ++){
            for (int i = 0; i < 3; i ++){
                for (int j = 0; j < 3; j ++){
                    cam_para_table>>temp1;
                    cam(i, j) = temp1;
                }
                cam_para_table>>temp1;
                campara_temp.get_translation()[i] = temp1/1000.0;
            }
            campara_temp.get_rotation() = cam;
            campara.push_back(campara_temp);
        }
        cout << "Camera relative pose loaded: translation and rotoation: " << endl;
        cout << campara_temp.get_translation() << endl;
        cout << campara_temp.get_rotation() << endl;

        return campara;//Tic
    }

    void image_ini_Callback(const sensor_msgs::ImageConstPtr& img_msg) {
        if ((!ini_one_circle_ && !ini_two_circle_) || isimage_ini_call)
            return;
        image_ini = img_msg;
        isimage_ini_call = true;
    }

    void viconCallback(const geometry_msgs::TransformStampedConstPtr& msg)
    {
        if (isviconcall)
            return;
        vicon_state = *msg;
        isviconcall = true;

    }

    // msg: pose info of the quadrotor (IMU frame) in the world frame Twb
    void pose_predicted_ekfCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        if (!isEKFattitudeInicall){ // receive the ini attitude info for initialization of PTAM from EKF
            if ((msg->pose.pose.position.x == 0) && // sign of such info
                (msg->pose.pose.position.y == 0) &&
                (msg->pose.pose.position.z == 0)){
                iniAttiEKF.w() = msg->pose.pose.orientation.w;
                iniAttiEKF.x() = msg->pose.pose.orientation.x;
                iniAttiEKF.y() = msg->pose.pose.orientation.y;
                iniAttiEKF.z() = msg->pose.pose.orientation.z;
                isEKFattitudeInicall = true;
            }
            return;
        }
        else if (!inipose_published)
            return;

        if (fabs(msg->pose.pose.position.x) > 1000)
            return;

        ispose_predicted_ekfcall = true;
        SE3<> Twb, Tcw;
        mPosePredictedlock.lock();
        pose_predicted_ekf_state_.header.stamp = msg->header.stamp;
        pose_predicted_ekf_state_.pose.pose.position.x = msg->pose.pose.position.x;
        pose_predicted_ekf_state_.pose.pose.position.y = msg->pose.pose.position.y;
        pose_predicted_ekf_state_.pose.pose.position.z = msg->pose.pose.position.z;
        PoseEKF_wi.get_translation()[0] = msg->pose.pose.position.x;
        PoseEKF_wi.get_translation()[1] = msg->pose.pose.position.y;
        PoseEKF_wi.get_translation()[2] = msg->pose.pose.position.z;

        for (unsigned int i = 0; i < pose_predicted_ekf_state_.pose.covariance.size(); i ++)
            pose_predicted_ekf_state_.pose.covariance[i] = msg->pose.covariance[i];

        // Also retrieve the scale factor, this should corresponed to the way we publish this infomation
        double scale = msg->pose.covariance[6];
        pose_predicted_ekf_state_.pose.covariance[6] = msg->pose.covariance[1];

        pose_predicted_ekf_state_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        pose_predicted_ekf_state_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        pose_predicted_ekf_state_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        pose_predicted_ekf_state_.pose.pose.orientation.z = msg->pose.pose.orientation.z;

        // pass the states to tracker
        if (use_ekf_pose && !tracker_->pose_ekf_get){
            tracker_->pose_ekf_get = true;
            ROS_INFO("POSE PREDICTION FROM EKF RECEIVED");
        }

        Twb.get_translation()[0] = pose_predicted_ekf_state_.pose.pose.position.x;
        Twb.get_translation()[1] = pose_predicted_ekf_state_.pose.pose.position.y;
        Twb.get_translation()[2] = pose_predicted_ekf_state_.pose.pose.position.z;

        Vector<4> q = makeVector(pose_predicted_ekf_state_.pose.pose.orientation.w,
                                 pose_predicted_ekf_state_.pose.pose.orientation.x,
                                 pose_predicted_ekf_state_.pose.pose.orientation.y,
                                 pose_predicted_ekf_state_.pose.pose.orientation.z);
        Matrix<3> rotation = tag::quaternionToMatrix(q);
        Twb.get_rotation() = rotation;
        Tcw = se3IMUfromcam.inverse() * Twb.inverse();

        pose_predicted_ekf_state_.pose.pose.position.x = Tcw.get_translation()[0];
        pose_predicted_ekf_state_.pose.pose.position.y = Tcw.get_translation()[1];
        pose_predicted_ekf_state_.pose.pose.position.z = Tcw.get_translation()[2];
        orientation_ekf_ = Tcw.get_rotation();
        scale_ekf_ = scale;// This is the scale of the pose estimation in the previous image frame

        
        mPosePredictedlock.unlock();

    }

    SE3<> tfs2se3(geometry_msgs::TransformStamped tfmsg){

        Vector<4> q = makeVector(tfmsg.transform.rotation.w, tfmsg.transform.rotation.x, tfmsg.transform.rotation.y, tfmsg.transform.rotation.z);
        Vector<3> t = makeVector(tfmsg.transform.translation.x, tfmsg.transform.translation.y, tfmsg.transform.translation.z);
        Matrix<3> transvicon = tag::quaternionToMatrix(q);// Rwc, Twc

        SE3<> se3;
        se3.get_rotation() = transvicon;
        se3.get_translation() = t;
        se3 = se3.inverse();//RTcw

        return se3;
    }

    SE3<> IMU2camWorldfromQuat(Eigen::Quaternion<double> atti){// use ini attitude info from EKF node to ini ptam pose
        Vector<4> attivec = makeVector(atti.w(), atti.x(), atti.y(), atti.z());//Rw1i
        Matrix<3> iniOrientationEKF;
        iniOrientationEKF = tag::quaternionToMatrix(attivec);

        Matrix<3> roll = TooN::Data(1.0, 0, 0,//Rww1, because the roll and pitch angles are in
                              0, -1, 0, // a world frame which pointing downward.
                              0, 0, -1);

        SE3<> camWorld = SE3<>();
        Matrix<3> rotation;
        if (tracker_->attitude_get)
            rotation = iniOrientationEKF; //
        else
            rotation = roll * iniOrientationEKF;//Rwi = Rww1*Rw1i
        camWorld.get_rotation() = rotation*se3IMUfromcam.get_rotation().get_matrix();//Rwc = (Rwi * Ric)

        Vector<3> twr = makeVector(0.0, 0.0, 0.198);// twc = twr + Rwr * trc
        Vector<3> twc = twr + rotation * se3IMUfromcam.get_translation();
        camWorld.get_translation()[0] = 0.0;//twc[0]; //twc
        camWorld.get_translation()[1] = 0.0;//twc[1];
        camWorld.get_translation()[2] = twc[2];

        camWorld = camWorld.inverse();//Tcw

        cout<< "TCW INITIALIZED. TWC: " << twc[0]<< ", " << twc[1]<< ", " << twc[2]<<endl;
        return camWorld;
    }

    // for EKF, we publish Tcw
    void publishPosewithCovariance(ros::Time stamp)
    {
        SE3<> camPose = camPose4pub;//Tcw

        if (true) {
            geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped);//Tcw
            pose->header.stamp = stamp;
            pose->header.frame_id = "/ptam_world_cov";

            pose->pose.pose.position.x = camPose.get_translation()[0];
            pose->pose.pose.position.y = camPose.get_translation()[1];
            pose->pose.pose.position.z = camPose.get_translation()[2];
            quat_from_so3(pose->pose.pose.orientation, camPose.get_rotation());

            TooN::Matrix<6> covar = tracker_->GetPoseCovariance();
            for (unsigned int i = 0; i < pose->pose.covariance.size(); i++){

                pose->pose.covariance[i] = sqrt(fabs(covar[i % 6][i / 6]));
//                cout << pose->pose.covariance[i] << ", ";
            }
            pose->pose.covariance[1] = tracker_->GetCurrentKeyFrame().dSceneDepthMedian;

            SE3<> roboPose = se3IMUfromcam * camPose;//Twi = (Tic * Tcw).inv
            roboPose = roboPose.inverse();
            cout << "POSITION DIFF: "<< PoseEKF_wi.get_translation()[0]-roboPose.get_translation()[0] << ", " <<
                    PoseEKF_wi.get_translation()[1] - roboPose.get_translation()[1]<< ", " <<
                    PoseEKF_wi.get_translation()[2] - roboPose.get_translation()[2] << endl;
            geometry_msgs::Quaternion quatRobo;
            quat_from_so3(quatRobo, roboPose.get_rotation());//Riw

            cam_posewithcov_pub_.publish(pose);

            geometry_msgs::TransformStampedPtr quadpose(new geometry_msgs::TransformStamped);// for ros nodelets, publish ptr.
            quadpose->header.stamp = stamp;
            quadpose->header.frame_id = "/ptam_world";
            quadpose->transform.translation.x = roboPose.get_translation()[0];
            quadpose->transform.translation.y = roboPose.get_translation()[1];
            quadpose->transform.translation.z = roboPose.get_translation()[2];
            quat_from_so3(quadpose->transform.rotation, roboPose.get_rotation());
            quad_pose_pub_.publish(quadpose);

            logPose(stamp, roboPose);
        }
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& rgb_img_msg) {
        if (lastDepth_) {
            imageCallback(rgb_img_msg, lastDepth_);
            lastDepth_.reset();
        }
    }
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_img_msg) {
        lastDepth_ = depth_img_msg;
    }
    // The image callback for the other cameras.
    // Pass the received image to the master callback function
    // Cannot avoid adding more such stupid functions when using more cameras
    // Tricky is how to retrieve the synchronised dual images
    void rgbCallback1(const sensor_msgs::ImageConstPtr& rgb_img_msg) {
        if ( lastDepth1_ ){
            secondcamlock.lock();
            img_secmsg = rgb_img_msg;
            secondimgcame = true;
            secondcamlock.unlock();
        }
    }
    void depthCallback1(const sensor_msgs::ImageConstPtr& depth_img_msg) {
        secondcamlock.lock();
        lastDepth1_ = depth_img_msg;
        secondcamlock.unlock();
        secondepthcame = true;
    }
    void rgbCallback2(const sensor_msgs::ImageConstPtr& rgb_img_msg) {
        if ( lastDepth2_ ){
            thirdcamlock.lock();
            img_3 = rgb_img_msg;
            thirdimgcame = true;
            thirdcamlock.unlock();
        }
    }
    void depthCallback2(const sensor_msgs::ImageConstPtr& depth_img_msg) {
        thirdcamlock.lock();
        lastDepth2_ = depth_img_msg;
        thirdcamlock.unlock();
        thirdepthcame = true;
    }

    // The image callback function for the master camera/ downward looking camera
    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg = sensor_msgs::ImageConstPtr()) {
        if (isPTAMshouldstop) {
            cout << "SLAM SHOULD STOP ASSUMING OUTLIERS OR LANDING ALREADY!" << endl;
            //return;
        }

        // TODO: synchronise multi images, could be simply assuming that their stamps are close to each other
        // main function should start when all imgs have come.
        ros::Time msg_stamp = img_msg->header.stamp;
        frame_stamp_ = img_msg->header.stamp.toSec();

        if (depth_msg){
            // Check image size:
            if (depth_msg->height != img_msg->height || depth_msg->width != img_msg->width) {
                ROS_ERROR_STREAM("error: rgb and depth image dimensions do not agree");
                return;
            }
            // Check depth type:
            if (depth_msg->encoding.compare("16SC1") != 0 && depth_msg->encoding.compare("16UC1") != 0) {
                ROS_ERROR_STREAM("error: ignoring depth image of unknown encoding: " << depth_msg->encoding);
                return;
            }
            // Create cv::Mat wrapped around raw data: no copying involved
//        cv::Mat depthMat(depth_img_msg->height, depth_img_msg->width, CV_16UC1, (unsigned char*) &depth_img_msg->data[0]);
        }

        convertImage(img_msg, 0);
        if (depth_msg) convertImage(depth_msg, 3);

        // TODO: better sync between two cams, consider the case that the second imgcb is slightly later than the first one.
        cursecimg_good = false;
        curthirdimg_good = false;
        if ( isdualcam )// && isFinishIniPTAMwithcircle)
        {
            if ( secondimgcame && secondepthcame ) {
                double imginterval = fabs(img_msg->header.stamp.toSec()-img_secmsg->header.stamp.toSec());
                if ( imginterval < intervalmax_imgdual)
                {
                    cout << "got img from secondcam..." << endl;
                    secondcamlock.lock();
                    convertImage(img_secmsg, 1);
                    convertImage(lastDepth1_, 4);
                    cout << "proccessed img from secondcam..." << endl;
                    secondimgcame = false;
                    secondepthcame = false;
                    secondcamlock.unlock();
                    cursecimg_good = true;// only when img is synchronised and ptam ini already
                }
            }
            if ( thirdimgcame && thirdepthcame ) {
                double imginterval = fabs(img_msg->header.stamp.toSec()-img_3->header.stamp.toSec());
                if ( imginterval < intervalmax_imgdual)
                {
                    thirdcamlock.lock();
                    convertImage(img_secmsg, 2);
                    convertImage(lastDepth2_, 5);
                    thirdimgcame = false;
                    thirdepthcame = false;
                    thirdcamlock.unlock();
                    curthirdimg_good = true;// only when img is synchronised and ptam ini already
                }
            }
        }

        // TODO: pass IMU attitude information to the ini module
        ros::Time time1 = ros::Time::now();

        if (!isdualcam) {
            if (!depth_msg)
                tracker_->TrackFrame(frameBW_);
            else
                tracker_->TrackFrame(frameRGB_, frameDepth_, rgbIsBgr_);
        } else
        {
            std::vector<CVD::Image<CVD::Rgb<CVD::byte> > > RGBimages;
            std::vector<CVD::Image<uint16_t> > DepthImages;
            std::vector<int> adcamIndex;
            RGBimages.push_back(frameRGB_);
            DepthImages.push_back(frameDepth_);

            if (cursecimg_good) {
                RGBimages.push_back(frameRGB_sec);
                DepthImages.push_back(frameDepth_sec);
                adcamIndex.push_back(0);
            }
            else
                return; // currently, force using dual kinects
            if (curthirdimg_good) {
                RGBimages.push_back(frameRGB_third);
                DepthImages.push_back(frameDepth_third);
                adcamIndex.push_back(1);
            }

            tracker_->TrackFrame(RGBimages, DepthImages, adcamIndex);
        }

        if (!isFinishIniPTAMwithcircle){
            camPoselast = tracker_->GetCurrentPose();
            camPosethis = camPoselast;
            camPose4pub = camPoselast;
        } else{
            camPosethis = tracker_->GetCurrentPose();
            camPose4pub = camPosethis;
        }
        isFinishIniPTAMwithcircle = true;

        ros::Time time2 = ros::Time::now();
        ros::Duration time3 = time2 - time1;

        cout << tracker_->GetMessageForUser() << endl;

        // ignore those too low information, useful when landing
        // also currently, when outliers happen, should stop PTAM for safty.
        if (//(ini_one_circle_ && tracker_->GetCurrentPose().inverse().get_translation()[2] < 0.3)
                (sqrt((camPosethis.get_translation()-camPoselast.get_translation())*
                        (camPosethis.get_translation()-camPoselast.get_translation()))>0.5))
        {
            isPTAMshouldstop = true;
            // camposelast keep still, just publish it
            camPose4pub = camPoselast;
        }
        else
            camPoselast = camPosethis;

        cout << camPosethis.get_translation() << endl << camPosethis.get_rotation().get_matrix() << endl;

        logPose(img_msg->header.stamp, SE3<>());

        if (sendvisual){// && !isPTAMshouldstop){
            if (cam_marker_pub_.getNumSubscribers() > 0 || point_marker_pub_.getNumSubscribers() > 0)
                map_viz_->publishMapVisualization(map_.get(),tracker_.get(),cam_marker_pub_,point_marker_pub_,
                                                  world_frame_, cam_marker_pub_sec, isdualcam);

            // broadcast tf for rviz
//            tf::TransformBroadcaster br;
//            tf::Transform transform;
//            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//            tf::Quaternion q;
//            q.setRPY(3.14159265/2.0, 3.14159265, 3.14159265/2.0);
//            transform.setRotation(q);
//            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame_, world_frame_));

            // TODO: use another thread to manage the point cloud, and publish it for Octomap-like mapper
            if (depth_msg && (vis_pointcloud_pub_.getNumSubscribers() > 0))
                map_viz_->publishPointCloud(map_.get(), tracker_.get(), vis_pointcloud_pub_, world_frame_,
                                            vis_publish_interval_, vis_pointcloud_step_, cellSize);
            if (depth_msg && (vis_crtpointcloud_pub_.getNumSubscribers() > 0))
                map_viz_->publishCrtPointCloud(map_.get(), tracker_.get(),
                                               vis_crtpointcloud_pub_, vis_crtpointcloud_pub_sec, world_frame_);
        }

        if (map_maker_->imageInputCount < 100)
            map_maker_->imageInputCount ++;
        else
            map_maker_->imageInputCount = 0;

        if (show_debug_image_) {
            cv::Mat rgb_cv(480, 640, CV_8UC3, frameRGB_.data());
            map_viz_->renderDebugImage(rgb_cv,tracker_.get(),map_.get());
            cv::imshow("forward",rgb_cv);
            cv::waitKey(10);
//            static int savedebugimg = 0;
//            if (savedebugimg == 0){
//                cv::imwrite("inimagedebug.jpg", rgb_cv);
//                savedebugimg = 1;
//            }
            if (cursecimg_good){
                cv::Mat rgb_cv(480, 640, CV_8UC3, frameRGB_sec.data());
                map_viz_->renderDebugImageSec(rgb_cv,tracker_.get(),map_.get());
                cv::imshow("side0",rgb_cv);
                cv::waitKey(10);
            }
        }
        cout << "Main image callback done." << endl;
    }

private:
    void convertImage(const sensor_msgs::ImageConstPtr& img_msg, int camnum = 1) {
        // convert image message to CVD image
        ImageRef img_size(img_msg->width,img_msg->height);
        if (img_msg->encoding.compare("rgb8") == 0 || img_msg->encoding.compare("bgr8") == 0) {
            switch (camnum){
            case 0:
                memcpy(frameRGB_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                break;
            case 1:
                memcpy(frameRGB_sec.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                break;
            case 2:
                memcpy(frameRGB_third.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                break;
            }

            if (img_msg->encoding.compare("rgb8") == 0)
                rgbIsBgr_ = false;
            else
                rgbIsBgr_ = true;
        } else if (img_msg->encoding.compare("mono8") == 0) {
            switch (camnum){
            case 0:
                memcpy(frameBW_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                CVD::convert_image(frameBW_,frameRGB_);
                break;
            case 1:
                memcpy(frameBW_sec.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                CVD::convert_image(frameBW_sec,frameRGB_sec);
                break;
            }

        } else if (img_msg->encoding.compare("16UC1") == 0){
            switch (camnum) {
            case 3:
                memcpy(frameDepth_.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                break;
            case 4:
                memcpy(frameDepth_sec.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                break;
            case 5:
                memcpy(frameDepth_third.data(),&(img_msg->data[0]),img_msg->step*img_size.y);
                break;
            }
            // cv::Mat depth_cv(480, 640, CV_16UC1, frameDepth_.data());
            // cv::imwrite("depth.jpg", depth_cv);
        } else {
            std::cerr << "error: ignoring image of unknown encoding: " << img_msg->encoding << std::endl;
            return;
        }
    }

    // change to log quadrotor pose, not camera pose
    void logPose(ros::Time msgtime, SE3<> roboPose)
    {
        SE3<> camPose = tracker_->GetCurrentPose().inverse();

        if (write_pos_) {//write_pos_
            btMatrix3x3 R(btScalar(camPose.get_rotation().get_matrix()(0,0)), btScalar(camPose.get_rotation().get_matrix()(0,1)), btScalar(camPose.get_rotation().get_matrix()(0,2)),
                          btScalar(camPose.get_rotation().get_matrix()(1,0)), btScalar(camPose.get_rotation().get_matrix()(1,1)), btScalar(camPose.get_rotation().get_matrix()(1,2)),
                          btScalar(camPose.get_rotation().get_matrix()(2,0)), btScalar(camPose.get_rotation().get_matrix()(2,1)), btScalar(camPose.get_rotation().get_matrix()(2,2)));
            btScalar roll1, pitch1, yaw1;
            R.getEulerYPR(yaw1, pitch1, roll1);
            roll1 = roll1 * 180.0 / M_PI;
            pitch1 = pitch1 * 180.0 / M_PI;
            yaw1 = yaw1 * 180.0 / M_PI;

            geometry_msgs::Quaternion quat;
            quat_from_so3(quat, camPose.get_rotation());

            // write to fake the loop closure moment
            static int looplognum = 0;
            if (debugloop.is_open()){
                if (tracker_->debugmarkLoopDetected){
                    looplognum ++;
                    if (looplognum > 1){
                        tracker_->debugmarkLoopDetected = false;
                        looplognum = 0;
                    }
                    debugloop <<msgtime.toSec() << " "  // for offline debuge
                             << camPose.get_translation()[0] <<  " "
                             << camPose.get_translation()[1] <<  " "
                             << camPose.get_translation()[2] <<  " "
                             << "\n";
                    for (int i = 0; i < 3; i ++){
                        for (int j =0; j < 3; j ++)
                            debugloop << camPose.get_rotation().get_matrix()(i, j) << " ";
                        debugloop << "\n" ;
                    }
                    debugloop << roll1 << " " << pitch1 << " " << yaw1 << "\n";
                    debugloop << quat.w << " " << quat.x << " " << quat.y << " " << quat.z << "\n";
                }
            }

            if (pos_log_.is_open()){
                pos_log_ <<msgtime.toSec() << " "  // for offline debuge
                           << camPose.get_translation()[0] <<  " "
                           << camPose.get_translation()[1] <<  " "
                           << camPose.get_translation()[2] <<  " "
                           << "\n";
                for (int i = 0; i < 3; i ++){
                    for (int j =0; j < 3; j ++)
                        pos_log_ << camPose.get_rotation().get_matrix()(i, j) << " ";
                    pos_log_ << "\n" ;
                }
                pos_log_ << roll1 << " " << pitch1 << " " << yaw1 << "\n";
                pos_log_ << quat.w << " " << quat.x << " " << quat.y << " " << quat.z << "\n";
            }
        }
    }

    image_transport::Subscriber rgbSubscriber_;
    image_transport::Subscriber depthSubscriber_;
    image_transport::Subscriber rgbSubscriber1_;
    image_transport::Subscriber depthSubscriber1_;
    image_transport::Subscriber rgbSubscriber2_;
    image_transport::Subscriber depthSubscriber2_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_sec; // the second camera image
    image_transport::Subscriber image4ini_sub_;// there would be only one or two images for ini ptam
    sensor_msgs::ImageConstPtr image_ini;// the one image from another ros node.

    sensor_msgs::ImageConstPtr lastDepth_;
    sensor_msgs::ImageConstPtr lastDepth1_;
    sensor_msgs::ImageConstPtr lastDepth2_;
    string image_;
    string image_sec;
    sensor_msgs::ImageConstPtr img_secmsg;// img msg from the second camera
    sensor_msgs::ImageConstPtr img_3;// img msg from the third camera
    bool cursecimg_good;
    bool curthirdimg_good;
    string pose_;
    bool sendvisual;
    bool use_circle_ini_;//use autonomous initialization method, may not noly use circle.
    ofstream pos_log_;
    ofstream debugloop;
    ofstream timecost;
    string ini_method_;
    bool ini_kinect_;
    bool ini_one_circle_;
    bool ini_ground_;
    bool ini_two_circle_;
    bool isFinishIniPTAMwithcircle;// when finish ini ptam with pose and image, do normal ptam
    bool isflying;// When quadrotor flying, constrain its possible attitude estimate angles.
    bool istrackPad;
    SE3<> se3IMUfromcam;
    vector<SE3<> > se3cam1fromcam2;
    string cam_para_path;
    string cam_para_path_second;// the second camera
    string ref_img_path;
    bool isPTAMshouldstop;// only used when landing near ground

    ros::Subscriber sub_attitude_;
    ros::Subscriber sub_pose_ekf_;//subscribe to pose prediction from EKF
    ros::Subscriber sub_pose_simplenode_;//subscribe to pose from anothor node for ini ptam, together with images
    ros::Publisher ini_circle_pub_;
    ros::Publisher cam_posewithcov_pub_;
    ros::Publisher quad_pose_pub_;
    ros::Publisher landingpad_marker_pub_;
    ros::Publisher landingpad_inipose_pub_;// two different headers used in setpoint node
    ros::Publisher landingpad_finalpose_pub_;
    ros::Publisher cam_marker_pub_sec;// for dual camera image

    geometry_msgs::TransformStamped vicon_state;
    geometry_msgs::PoseWithCovarianceStamped pose_predicted_ekf_state_;
    SE3<> PoseEKF_wi;
    SO3<> orientation_ekf_;
    double scale_ekf_;
    SE3<> camPoselast;// Tcw in the last frame
    SE3<> camPosethis;// and this frame

    bool isattitudecall;
    bool isviconcall;// pose together with image/s for ini ptam
    bool isimage_ini_call;// the image/s for ini ptam
    bool inipose_published;//pose info for initializing EKF published?

    bool useEKFiniAtti;// use attitude info from EKF node?
    bool use_ekf_pose;
    bool ispose_predicted_ekfcall;//received pose prediction from EKF?
    bool isEKFattitudeInicall;// use attitude initialization from EKF
    Eigen::Quaternion<double> iniAttiEKF;//

    boost::mutex mviconlock, mAttitudelock, mPosePredictedlock;
    boost::mutex secondcamlock;
    boost::mutex thirdcamlock;

    CVD::Image<CVD::Rgb<CVD::byte> > frameRGB_,frameRGB_sec, frameRGB_third;
    CVD::Image<CVD::byte> frameBW_, frameBW_sec;
    CVD::Image<uint16_t> frameDepth_, frameDepth_sec, frameDepth_third;

    bool secondimgcame;// dual camera case
    bool thirdimgcame;
    bool secondepthcame;// dual camera case
    bool thirdepthcame;
};


// Register this plugin with pluginlib.  Names must match nodelet_...xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(idSLAM, idslamnodelet,
                        idslamnodelet, nodelet::Nodelet);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dualmono_nodelet");

    idslamnodelet nodelet;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

}
