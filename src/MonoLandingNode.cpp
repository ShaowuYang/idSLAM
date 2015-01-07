 /*
 * PTAMNode.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: yang, scherer
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <highgui.h>

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

#include "BasePTAMNode.h"
#include "MapVisualization.h"

#include <pximu/AttitudeData.h>
#include <pximu/RawIMUData.h>

#include "conversions/conversions.h"

using namespace CVD;
using namespace std;

class MonoLandingNode : public BasePTAMNode {
public:
    MonoLandingNode() :
        BasePTAMNode()
    {
        if(!nh_private_.getParam("image", image_))
            image_ = "/camera/image_raw";
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
            ini_method_ = "two_circle";
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

        se3IMUfromcam = Load_cam_para();

        // yang, load ref img of the landing pad, do similar 4 scale sampling as ptam
        tracker_->load_reflandingpad(ref_img_path);

        pos_log_.open("ptam.log");
        if (write_pos_)
            cout <<"logfile open?: " << pos_log_.is_open() << endl;
        if (pos_log_.is_open()){
            pos_log_.setf(std::ios::fixed, std::ios::floatfield);
            pos_log_.precision(10);
        }

        image_transport::ImageTransport it(nh_);
        image_sub_ = it.subscribe(image_, 1, boost::bind(&MonoLandingNode::imageCallback, this, _1));

        image4ini_sub_ = it.subscribe("/simple_landing/image_ini_ptam", 1, boost::bind(&MonoLandingNode::image_ini_Callback, this, _1));
        sub_attitude_ = nh_.subscribe<pximu::AttitudeData>("/attitude", 1, boost::bind(&MonoLandingNode::attitudeCallback, this, _1));
        sub_pose_ekf_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/ssf_core/pose_predicted_ekf", 1, boost::bind(&MonoLandingNode::pose_predicted_ekfCallback, this, _1));

        sub_pose_simplenode_ = nh_.subscribe<geometry_msgs::TransformStamped>("/simple_landing/vicon_pose_ptam", 1, boost::bind(&MonoLandingNode::viconCallback, this, _1));

        ini_circle_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/ptam/ini_circle",1);
        cam_posewithcov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ptam_world_cov",1);
        quad_pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(pose_,1);

        landingpad_inipose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/landingpad_inipose",1);
        landingpad_finalpose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/landingpad_finalpose",1);
        landingpad_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ptam/landingpad", 1);

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
    }

    ~MonoLandingNode()
    {
        if (pos_log_.is_open())
            pos_log_.close();
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

    void image_ini_Callback(const sensor_msgs::ImageConstPtr& img_msg) {
        if ((!ini_one_circle_ && !ini_two_circle_) || isimage_ini_call)
            return;
//        if (ini_one_circle_)
//            convertImage(img_msg);
        image_ini = img_msg;
        isimage_ini_call = true;
    }

    void viconCallback(const geometry_msgs::TransformStampedConstPtr& msg)
    {
        if (isviconcall)
            return;
        vicon_state = *msg;
//        if (vicon_state.transform.translation.x != 0)
        isviconcall = true;

    }

    // Attitude data from IMU
    void attitudeCallback(const pximu::AttitudeDataConstPtr& msg)
    {
        static int ini_att = 0;
        static int ini_att_limit = 30;
        static double roll = 0;
        static double pitch = 0;

        if ((ini_att < ini_att_limit) && !isattitudecall){//initialize PTAM using attitude info when quadrotor stand still
            roll += msg->roll;
            pitch += msg->pitch;
            ini_att ++;
        }
        else{
            ini_att = 0;
            if (!isattitudecall){
                mAttitudelock.lock();
                attitude_data_.header.stamp = msg->header.stamp;

                attitude_data_.roll = roll / ini_att_limit;
                attitude_data_.pitch = pitch / ini_att_limit;

                attitude_data_.yaw        = msg->yaw;
                attitude_data_.rollspeed  = msg->rollspeed;
                attitude_data_.pitchspeed = msg->pitchspeed;
                attitude_data_.yawspeed   = msg->yawspeed;
                mAttitudelock.unlock();
                cout << "attitude from imu: " << attitude_data_.roll << ", " << attitude_data_.pitch << endl;
            }
            else {
                if (!inipose_published) //if pose for ini external EKF hasn't been published, keep this filtered pose.
                    return;
                mAttitudelock.lock();
                attitude_data_.header.stamp = msg->header.stamp;
                attitude_data_.roll       = msg->roll;
                attitude_data_.pitch      = msg->pitch;
                attitude_data_.yaw        = msg->yaw;
                attitude_data_.rollspeed  = msg->rollspeed;
                attitude_data_.pitchspeed = msg->pitchspeed;
                attitude_data_.yawspeed   = msg->yawspeed;
                mAttitudelock.unlock();
            }
            isattitudecall = true;
        }
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

//        Eigen::Quaternion<double> quat;
//        quat.w = msg->pose.pose.orientation.w;
//        quat.x = msg->pose.pose.orientation.x;
//        quat.y = msg->pose.pose.orientation.y;
//        quat.z = msg->pose.pose.orientation.z;
//        quat.normalize();
        pose_predicted_ekf_state_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        pose_predicted_ekf_state_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        pose_predicted_ekf_state_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        pose_predicted_ekf_state_.pose.pose.orientation.z = msg->pose.pose.orientation.z;

        // pass the states to tracker
        if (use_ekf_pose && !tracker_->pose_ekf_get){
            tracker_->pose_ekf_get = true;
            ROS_INFO("POSE PREDICTION FROM EKF RECEIVED");
        }
//        ROS_INFO("POSE PREDICTION FROM EKF RECEIVED, twi: (%f, %f, %f) ", pose_predicted_ekf_state_.pose.pose.position.x,
//                 pose_predicted_ekf_state_.pose.pose.position.y,
//                 pose_predicted_ekf_state_.pose.pose.position.z);

        Twb.get_translation()[0] = pose_predicted_ekf_state_.pose.pose.position.x;
        Twb.get_translation()[1] = pose_predicted_ekf_state_.pose.pose.position.y;
        Twb.get_translation()[2] = pose_predicted_ekf_state_.pose.pose.position.z;

        Vector<4> q = makeVector(pose_predicted_ekf_state_.pose.pose.orientation.w,
                                 pose_predicted_ekf_state_.pose.pose.orientation.x,
                                 pose_predicted_ekf_state_.pose.pose.orientation.y,
                                 pose_predicted_ekf_state_.pose.pose.orientation.z);
        Matrix<3> rotation = tag::quaternionToMatrix(q);
        Twb.get_rotation() = rotation;
//        cout << rotation(0, 0) << ", " << rotation(0,1) << ", " << rotation(0,2) << endl;
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
//        se3.get_rotation() = se3.get_rotation().inverse();//Rcw, Tcw
//        se3.get_translation() = se3.get_rotation() * (-t);
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
//        cout<< camWorld.get_rotation().get_matrix()(2,2)<<endl;
        return camWorld;
    }

    // get Tcw
    SE3<> imu2camWorld(pximu::AttitudeData attitude_data){
        Matrix<3> Rroll =  Data(1.0, 0, 0,//roll Rot(-roll)T=Rot(roll)
                              0, cos(attitude_data.roll), -sin(attitude_data.roll),
                              0, sin(attitude_data.roll), cos(attitude_data.roll));
        Matrix<3> Rpitch = Data(cos(attitude_data.pitch), 0, -sin(attitude_data.pitch),//pitch Rot(-pitch)T=Rot(pitch)
                              0, 1.0, 0,
                              sin(attitude_data.pitch), 0, cos(attitude_data.pitch));

        Matrix<3> roll = Data(1.0, 0, 0,//Rww1, because the roll and pitch angles are in
                              0, -1, 0, // a world frame which pointing downward.
                              0, 0, -1);

        SE3<> camWorld = SE3<>();
        Matrix<3> rotation = roll * Rpitch * Rroll; //Rwr*Rrc-->Rwc
        camWorld.get_rotation() = rotation*se3IMUfromcam.get_rotation().get_matrix();

        Vector<3> twr = makeVector(0.0, 0.0, 0.198);// twc = twr + Rwr * trc
        Vector<3> twc = twr + rotation * se3IMUfromcam.get_translation();
        camWorld.get_translation()[0] = 0.0;//twc[0]; //twc
        camWorld.get_translation()[1] = 0.0;//twc[1]; // TODO: find out the bug why must use 0
        camWorld.get_translation()[2] = twc[2];

        camWorld = camWorld.inverse();//Tcw

        cout<< "TCW INITIALIZED FROM IMU ATT. TWC: " << twc[0]<< ", " << twc[1]<< ", " << twc[2]<<endl;
        return camWorld;
    }

    // for EKF, we publish Tcw
    void publishPosewithCovariance(ros::Time stamp)
    {
//        SE3<> camPose = tracker_->GetCurrentPose();//Tcw
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

//            cout << "PTAM pose, Tcw: " << endl;
//            cout << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z << endl;
//            cout << pose->pose.pose.orientation.w << ", " << pose->pose.pose.orientation.x << ", "
//                 << pose->pose.pose.orientation.y << ", " << pose->pose.pose.orientation.z << endl;

            SE3<> roboPose = se3IMUfromcam * camPose;//Twi = (Tic * Tcw).inv
            roboPose = roboPose.inverse();
            ROS_INFO("POSITION DIFF: (%f, %f, %f)", PoseEKF_wi.get_translation()[0]-roboPose.get_translation()[0],
                     PoseEKF_wi.get_translation()[1] - roboPose.get_translation()[1], PoseEKF_wi.get_translation()[2] - roboPose.get_translation()[2]);
            geometry_msgs::Quaternion quatRobo;
            quat_from_so3(quatRobo, roboPose.get_rotation());//Riw
//            cout << "PTAM pose, Twi: " << endl;
//            cout << roboPose.get_translation()[0] << ", " << roboPose.get_translation()[1] << ", " << roboPose.get_translation()[2] << endl;
//            cout << quatRobo.w << ", " << quatRobo.x << ", "
//                 << quatRobo.y << ", " << quatRobo.z << endl;


//            cout << "Pose Covariance: " << endl;
//            cout << pose->pose.covariance[0] << ", " << pose->pose.covariance[1] << ", " << pose->pose.covariance[2] << endl;
//            cout << pose->pose.covariance[3] << ", " << pose->pose.covariance[4] << ", " << pose->pose.covariance[5] << endl;
//            cout << pose->pose.covariance[6] << ", " << pose->pose.covariance[7] << ", " << pose->pose.covariance[8] << endl;

            cam_posewithcov_pub_.publish(pose);

            geometry_msgs::TransformStamped quadpose;
            quadpose.header.stamp = stamp;
            quadpose.header.frame_id = "/ptam_world";
            quadpose.transform.translation.x = roboPose.get_translation()[0];
            quadpose.transform.translation.y = roboPose.get_translation()[1];
            quadpose.transform.translation.z = roboPose.get_translation()[2];
            quat_from_so3(quadpose.transform.rotation, roboPose.get_rotation());
            quad_pose_pub_.publish(quadpose);

            logPose(stamp, roboPose);
        }
    }

    void publish_inilandingpadpose(ros::Time stamp)
    {
        // keep publish the first pose detected until finally refined in mapmaker
        static TooN::Vector<3> mpadPose = tracker_->iniPadCenterWorld;//Twp

        geometry_msgs::TransformStamped padpose;
        padpose.header.stamp = stamp;
        padpose.header.frame_id = "/landingpad";
        padpose.transform.translation.x = mpadPose[0];
        padpose.transform.translation.y = mpadPose[1];
        padpose.transform.translation.z = mpadPose[2];
        padpose.transform.rotation.x = padpose.transform.rotation.y = padpose.transform.rotation.z = 0;
        padpose.transform.rotation.w = 1;
        landingpad_inipose_pub_.publish(padpose);
    }

    void publish_finallandingpadpose(ros::Time stamp)
    {
        static TooN::Vector<3> mpadPose = tracker_->mPadCenterWorld;//Twp

        geometry_msgs::TransformStamped padpose;
        padpose.header.stamp = stamp;
        padpose.header.frame_id = "/landingpad";
        padpose.transform.translation.x = mpadPose[0];
        padpose.transform.translation.y = mpadPose[1];
        padpose.transform.translation.z = mpadPose[2];
        padpose.transform.rotation.x = padpose.transform.rotation.y = padpose.transform.rotation.z = 0;
        padpose.transform.rotation.w = 1;
        landingpad_finalpose_pub_.publish(padpose);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
        if (isPTAMshouldstop) {
            ROS_INFO("PTAM SHOULD STOP ASSUMING OUTLIERS OR LANDING ALREADY!");
//            return;
        }
        static int img_count = 0;
        if (img_count == 1){// image frequency / 2 if rosbag 60fps
            img_count = 0;
//            if (!(use_circle_ini_ && !isFinishIniPTAMwithcircle))
                return;
        }
        img_count ++;

        ros::Time msg_stamp = img_msg->header.stamp;
//        cout << "image stamp: " << msg_stamp << endl;
        frame_stamp_ = img_msg->header.stamp.toSec();
        ROS_INFO("TIMESTAMP: %f", frame_stamp_);

        if (ini_one_circle_ && !isFinishIniPTAMwithcircle)
        {
            if ( !isimage_ini_call || !isviconcall) return;
            else{
                convertImage(image_ini);
                msg_stamp = image_ini->header.stamp;

                CVD::img_save(frameBW_, "inimage.jpg");
            }
        }
        else
            convertImage(img_msg);

        // pass image frame along to PTAM
        if (use_circle_ini_ && !isFinishIniPTAMwithcircle){
            // use ini pose and image from another node: simple_landing node
            if ((ini_one_circle_ || ini_two_circle_) && !tracker_->circle_pose_get ){
                tracker_->circle_pose_get = true;
                tracker_->se3CfromW = tfs2se3(vicon_state);// just need Twc
                tracker_->time_stamp_circle = vicon_state.header.stamp.toSec();
                tracker_->time_stamp_now = ros::Time::now().toSec();
            }

            // use ground information and IMU/EKF info.
            else if (ini_ground_ && !isEKFattitudeInicall && useEKFiniAtti){
                cout << "Waiting for attitude from EKF node..." << endl;
                return;
            }
            else if (ini_ground_ && !isattitudecall && !useEKFiniAtti){
                cout << "Waiting for attitude from IMU..." << endl;
                return;
            }
            else if (ini_ground_ && useEKFiniAtti && isEKFattitudeInicall ){
                if (!tracker_->attitude_get){

                    tracker_->se3IMU_camfromW = IMU2camWorldfromQuat(iniAttiEKF);//Tcw from Twi IMU calculated attitude

                    cout << "PTAM attitude got from EKF: "<< iniAttiEKF.w() <<", "
                                                << iniAttiEKF.x() <<", "
                                                << iniAttiEKF.y() <<", "
                                                << iniAttiEKF.z() <<endl;

                    tracker_->attitude_get = true;
                }
            }
            else if (ini_ground_ && !useEKFiniAtti && isattitudecall){
                if (!tracker_->attitude_get){
                    mAttitudelock.lock();
                    tracker_->se3IMU_camfromW = imu2camWorld(attitude_data_);//Twc from IMU calculated attitude
                    mAttitudelock.unlock();

                    cout << "PTAM pose got from IMU: "<< attitude_data_.roll <<", "
                            << attitude_data_.pitch<<endl;

                    tracker_->attitude_get = true;
                }
            }
        }

        if (use_ekf_pose && ispose_predicted_ekfcall) {
            mPosePredictedlock.lock();
            tracker_->se3CfromW_predicted_ekf.get_translation()[0] = pose_predicted_ekf_state_.pose.pose.position.x;
            tracker_->se3CfromW_predicted_ekf.get_translation()[1] = pose_predicted_ekf_state_.pose.pose.position.y;
            tracker_->se3CfromW_predicted_ekf.get_translation()[2] = pose_predicted_ekf_state_.pose.pose.position.z;
            tracker_->se3CfromW_predicted_ekf.get_rotation() = orientation_ekf_;
            tracker_->scale_ekf = scale_ekf_;// This is the scale of the pose estimation in the previous image frame
            mPosePredictedlock.unlock();
        }

        ros::Time time1 = ros::Time::now();


        // for debuging
//        static int published_num = 0;
//        if (published_num == 2)
//            return;

        tracker_->TrackFrame(frameBW_);

        if (!isFinishIniPTAMwithcircle){
            camPoselast = tracker_->GetCurrentPose();
            camPosethis = camPoselast;
            camPose4pub = camPoselast;
        }
        else{
            camPosethis = tracker_->GetCurrentPose();
            camPose4pub = camPosethis;
        }
        isFinishIniPTAMwithcircle = true;

        ros::Time time2 = ros::Time::now();
        ros::Duration time3 = time2 - time1;
//        ROS_INFO("PTAM time cost: %f", time3.toSec());

        if (use_circle_ini_)
            cout << tracker_->GetMessageForUser() << " " << endl;
        else
            cout << tracker_->GetMessageForUser() << endl;

        // ignore those too low information, useful when landing
        // also currently, when outliers happen, should stop PTAM for safty.
        if ((ini_one_circle_ && tracker_->GetCurrentPose().inverse().get_translation()[2] < 0.3)
                ||(sqrt((camPosethis.get_translation()-camPoselast.get_translation())*
                        (camPosethis.get_translation()-camPoselast.get_translation()))>0.5))
        {
            isPTAMshouldstop = true;
//            return;
            // camposelast keep still, just publish it
            camPose4pub = camPoselast;
        }
        else
            camPoselast = camPosethis;
        // publish current pose in map in different cases
        if ( (use_circle_ini_ && !use_ekf_pose) || (use_ekf_pose && isEKFattitudeInicall)){
                publishPose(msg_stamp);
                publishPosewithCovariance(msg_stamp);
                inipose_published = true;

                // yang, publish landing object pose to set-point node?
                if (tracker_->isLandingpadPoseGet){
                    publish_inilandingpadpose(msg_stamp);
                    if (!tracker_->isFinishPadDetection)
                        publish_finallandingpadpose(msg_stamp);
                } 
        }

        if (use_circle_ini_){
            static geometry_msgs::TransformStamped ptam_state;
            if (tracker_->ini_circle_finished)
                ptam_state.transform.rotation.w = 1;//only for state sign recognition
            else
                ptam_state.transform.rotation.w = 0;
            ini_circle_pub_.publish(ptam_state);
        }

        if (sendvisual && !isPTAMshouldstop){
            if (cam_marker_pub_.getNumSubscribers() > 0 || point_marker_pub_.getNumSubscribers() > 0)
                map_viz_->publishMapVisualization(map_,tracker_,cam_marker_pub_,point_marker_pub_);
            if ((landingpad_marker_pub_.getNumSubscribers() > 0) && tracker_->isLandingpadPoseGet)
                map_viz_->publishlandingpad(tracker_, landingpad_marker_pub_);
        }

        if (show_debug_image_) {
            cv::Mat rgb_cv(480, 640, CV_8UC3, frameRGB_.data());
            map_viz_->renderDebugImage(rgb_cv,tracker_,map_);
            cv::imshow("debug",rgb_cv);
            cv::waitKey(10);
            static int savedebugimg = 0;
            if (savedebugimg == 0){
                cv::imwrite("inimagedebug.jpg", rgb_cv);
                savedebugimg = 1;
            }

            if (false)//(istrackPad)
            {
                CVD::Image<CVD::Rgb<CVD::byte> > frameRef;
                CVD::Image<CVD::byte> frameRefbw;
                frameRefbw.resize(tracker_->GetRefFrame().aLevels[0].im.size());
                frameRef.resize(tracker_->GetRefFrame().aLevels[0].im.size());
                CVD::copy(tracker_->GetRefFrame().aLevels[0].im, frameRefbw);
                //            memcpy(frameRefbw.data(), tracker_->GetRefFrame().aLevels[0].im.data(), tracker_->GetRefFrame().aLevels[0].im.totalsize());
                CVD::convert_image(frameRefbw, frameRef);
                cv::Mat rgb_ref(frameRef.size().y, frameRef.size().x, CV_8UC3, frameRef.data());
                map_viz_->renderRefImage(rgb_ref, tracker_);
                cv::imshow("ref", rgb_ref);
                cv::waitKey(10);
            }

        }
//        isviconcall = false;
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

    // change to log quadrotor pose, not camera pose
    void logPose(ros::Time msgtime, SE3<> roboPose)
    {
        SE3<> camPose = roboPose;//tracker_->GetCurrentPose().inverse();

        if (write_pos_) {//write_pos_
//            geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped);
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

            if (pos_log_.is_open()){
//                pos_log_ << msgtime.toSec() << " "
                pos_log_ <<ros::Time::now().toSec() << " "  // for offline debuge
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

    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image4ini_sub_;// there would be only one or two images for ini ptam
    sensor_msgs::ImageConstPtr image_ini;// the one image from another ros node.

    string image_;
    string pose_;
    bool sendvisual;
    bool use_circle_ini_;//use autonomous initialization method, may not noly use circle.
    ofstream pos_log_;
    string ini_method_;
    bool ini_one_circle_;
    bool ini_ground_;
    bool ini_two_circle_;
    bool isFinishIniPTAMwithcircle;// when finish ini ptam with pose and image, do normal ptam
    bool isflying;// When quadrotor flying, constrain its possible attitude estimate angles.
    bool istrackPad;
    SE3<> se3IMUfromcam;
    string cam_para_path;
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

    geometry_msgs::TransformStamped vicon_state;
    pximu::AttitudeData attitude_data_;
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


    CVD::Image<CVD::Rgb<CVD::byte> > frameRGB_;
    CVD::Image<CVD::byte> frameBW_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mono_landing_node");

    MonoLandingNode node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

