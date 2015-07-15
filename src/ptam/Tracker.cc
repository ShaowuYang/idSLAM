// Copyright 2008 Isis Innovation Limited
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "MapPoint.h"
#include "TrackerData.h"
#include <cs_geometry/Conversions.h>

#include <cvd/utility.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <cvd/image_io.h>

#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <Eigen/Core>

#include <fstream>
#include <fcntl.h>


using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace ptam;

// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(Map &m, MapMaker &mm) :
    mCurrentKF(new KeyFrame()),
    mMap(m),
    mMapMaker(mm),
    mCamera(CameraModel::CreateCamera()),
    mRelocaliser(mMap)
{
    mCurrentKF->bFixed = false;
    for (int i = 0; i < AddCamNumber; i ++){
        std::auto_ptr<CameraModel> camera_temp (CameraModel::CreateCamera(i + 1));
        mCameraSec[i] = camera_temp;

        boost::shared_ptr<KeyFrame> kf_temp (new KeyFrame());
        mCurrentKFsec[i] = kf_temp;
        mCurrentKFsec[i]->bFixed = false;

        mse3CamFromWorldsec[i] = SE3<>();
    }
    GUI.RegisterCommand("Reset", GUICommandCallBack, this);
    GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
    GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);

    mpSBILastFrame = NULL;
    mpSBIThisFrame = NULL;
    for (int i = 0; i < AddCamNumber; i ++){
        mpSBILastFramesec[i] = NULL;
        mpSBIThisFramesec[i] = NULL;
    }

    // Most of the initialisation is done in Reset()
    Reset();

    use_circle_ini = false;//use circle initialise
    use_one_circle = false;
    use_two_circle = false;
    use_ground = false;
    ini_circle_finished = false;
    mse3CamFromWorld = SE3<>();
    mse3CamFromWorldPub = SE3<>();
    se3CfromW = SE3<>();
    se3CfromW_predicted_ekf = SE3<>();
    circle_pose_get = false;
    attitude_get = false;
    pose_ekf_get = false;
    isflying = false;

    istrackPad = false;
    isLandingpadPoseGet = false;
    isFinishPadDetection = false;

    time_last_detect = ros::Time::now();
    timecost_vo = 0;

    wls.clear();
    wls2.clear();
    tracking_map = false;
    trackerlogdebug.open("trackerlogdebug.log");
    if (trackerlogdebug.is_open()){
        trackerlogdebug.setf(std::ios::fixed, std::ios::floatfield);
        trackerlogdebug.precision(10);
    }
    trackerlog.open("trackerlog.log");
    if (trackerlog.is_open()){
        trackerlog.setf(std::ios::fixed, std::ios::floatfield);
        trackerlog.precision(10);
    }
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{
    mbDidCoarse = false;
    mbUserPressedSpacebar = false;
    mTrackingQuality = GOOD;
    mnLostFrames = 0;
    mnFailureFrames = 0;
    mbOldMaplocked = false;
    mdMSDScaledVelocityMagnitude = 0;
    mCurrentKF->dSceneDepthMean = 1.0;
    mCurrentKF->dSceneDepthSigma = 1.0;
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
    mCurrentKF->mMeasurements.clear();
    mnLastKeyFrameDropped = -20;
    mnFrame=0;
    mnKeyFrames=0;
    mv6CameraVelocity = Zeros;
    mbJustRecoveredSoUseCoarse = false;
    mUsingDualImg = false;
    mUseDualshould = false;
    debugmarkLoopDetected = false;
    boost::shared_ptr<KeyFrame> kf_temp (new KeyFrame());
    mGoodKFtoTrack= kf_temp;
    for (int i = 0; i < AddCamNumber; i ++){
        mCurrentKFsec[i]->dSceneDepthMean = 1.0;
        mCurrentKFsec[i]->dSceneDepthSigma = 1.0;
        mCurrentKFsec[i]->mMeasurements.clear();

        mv6CameraVelocitysec[i] = Zeros;
        mnLastKeyFrameDroppedsec[i] = -20;
        mnFramesec[i]=0;
        mnKeyFramessec[i]=0;

        boost::shared_ptr<KeyFrame> kf_temp (new KeyFrame());
        mGoodKFtoTracksec[i] = kf_temp;
    }
    ActiveAdCamIndex.clear();


    // Tell the MapMaker to reset itself..
    // this may take some time, since the mapmaker thread may have to wait
    // for an abort-check during calculation, so sleep while waiting.
    // MapMaker will also clear the map.
    mMapMaker.RequestReset();
    while(!mMapMaker.ResetDone())
#ifndef WIN32
        usleep(10);
#else
        Sleep(1);
#endif
}

void Tracker::UpdateImageSize(const CVD::ImageRef& size)
{
    if (mirSize.x == 0  && mirSize.y == 0) { // not yet initialized (before processing first image)
        mirSize = size;
        TrackerData::irImageSize = size;
        mCamera->SetImageSize(mirSize);
    } else if (mirSize.x != size.x || mirSize.y != size.y) {
        std::cerr << __PRETTY_FUNCTION__ << "error: image size changed." << std::endl;

        // update image size anyway.
        mirSize = size;
        TrackerData::irImageSize = size;
        mCamera->SetImageSize(mirSize);
    }
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::TrackFrame(Image<byte> &imFrame)
{
    cout << "Dual images made kfs1..." <<"\n";
    mUsingDualImg = false;
    mMessageForUser.str("");   // Wipe the user message clean

    UpdateImageSize(imFrame.size());

    // Take the input video image, and convert it into the tracker's keyframe struct
    // This does things like generate the image pyramid and find FAST corners
    mCurrentKF->mMeasurements.clear();
    mCurrentKF->MakeKeyFrame_Lite(imFrame);
    mCurrentKF->MakeKeyFrame_Rest();
    mCurrentKF->nSourceCamera = 0;

    initNewFrame();
    if(!trackMap()) {
        // If there is no map, try to make one.
        if (use_circle_ini){
            if (use_one_circle && circle_pose_get){ //&& (se3CfromW.inverse().get_translation()[2] > 0.4)){
                mse3CamFromWorld = se3CfromW;
                mse3CamFromWorldsec[0] = mse3Cam1FromCam2[0].inverse()*mse3CamFromWorld;
                mse3CamFromWorldPub = se3CfromW;
                mCurrentKF->se3CfromW = se3CfromW;
                //                    mCurrentKF->MakeKeyFrame_Rest();
                mMapMaker.InitFromOneCircle(*mCurrentKF, mse3CamFromWorld, 20, 3);
                ini_circle_finished = true;
            }
            else if (use_ground && attitude_get ){
                mse3CamFromWorld = se3IMU_camfromW;
                mse3CamFromWorldPub = se3IMU_camfromW;
                mCurrentKF->se3CfromW = se3IMU_camfromW;

                //                    mCurrentKF->MakeKeyFrame_Rest();
                mMapMaker.InitFromOneCircle(*mCurrentKF, mse3CamFromWorld, 20, 3);
            }
        }
        else if (use_kinect){
            mCurrentKF->se3CfromW = SE3<>();
            mMapMaker.InitFromRGBD(*mCurrentKF);
        }
        else
            TrackForInitialMap();
    }

    processGUIEvents();
};

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
// use dual image: sec img with known extrinsic param w.r.t the main img
void Tracker::TrackFrame(Image<byte> &imFrame, Image<byte> &imFramesec)
{
    mUsingDualImg = true;
    mMessageForUser.str("");   // Wipe the user message clean
    UpdateImageSize(imFrame.size());

    ros::Time timevobegin=ros::Time::now();

    // Take the input video image, and convert it into the tracker's keyframe struct
    // This does things like generate the image pyramid and find FAST corners
    mCurrentKF->mMeasurements.clear();
    mCurrentKF->MakeKeyFrame_Lite(imFrame);
    mCurrentKF->MakeKeyFrame_Rest();
    mCurrentKF->nSourceCamera = 0;
    // and the second camera img
    mCurrentKFsec[0]->mMeasurements.clear();
    mCurrentKFsec[0]->MakeKeyFrame_Lite(imFramesec, 1);
    mCurrentKFsec[0]->MakeKeyFrame_Rest();
    mCurrentKFsec[0]->nSourceCamera = 1;
    mCurrentKFsec[0]->mAssociateKeyframe = true;

    // dual image should be initialised individually
    initNewFrame();
    initNewFrame_sec();// the second current keyframe

    if(!trackMapDual()) {
        // If there is no map, try to make one.
        if (use_circle_ini){
            if (use_one_circle && circle_pose_get){ //&& (se3CfromW.inverse().get_translation()[2] > 0.4)){
                mse3CamFromWorld = se3CfromW;
                mse3CamFromWorldsec[0] = mse3Cam1FromCam2[0].inverse()*se3CfromW;
                mse3CamFromWorldPub = se3CfromW;
                mCurrentKF->se3CfromW = se3CfromW;
                //                    mCurrentKF->MakeKeyFrame_Rest();
                // if use InitFromOneCircleDual, changes should be made for kf id of the backend
//                mMapMaker.InitFromOneCircleDual(*mCurrentKF, *mCurrentKFsec,mse3CamFromWorld, 20, 3);
                mMapMaker.InitFromOneCircle(*mCurrentKF,mse3CamFromWorld, 20, 3);
                ini_circle_finished = true;
            }
            else if (use_ground && attitude_get ){
                mse3CamFromWorld = se3IMU_camfromW;
                mse3CamFromWorldPub = se3IMU_camfromW;
                mCurrentKF->se3CfromW = se3IMU_camfromW;

                //                    mCurrentKF->MakeKeyFrame_Rest();
//                mMapMaker.InitFromOneCircleDual(*mCurrentKF, *mCurrentKFsec, mse3CamFromWorld, 20, 3);
                mMapMaker.InitFromOneCircle(*mCurrentKF,mse3CamFromWorld, 20, 3);
            }
        }
        else
            TrackForInitialMap();
    }

    ros::Time timevoend = ros::Time::now();
    ros::Duration timevo = timevoend - timevobegin;
    timecost_vo = timevo.toSec();

    if (trackerlog.is_open()){

//                pos_log_ << msgtime.toSec() << " "
        trackerlog <<timevobegin.toSec() << " "
                   << timecost_vo <<  " "
                   << "\n";
    }

    processGUIEvents();
    mUsingDualImg = false;
};

void Tracker::TrackFrame(CVD::Image<CVD::byte> &imFrame, CVD::Image<uint16_t> &imFrameD)
{
    mMessageForUser.str("");   // Wipe the user message clean

    UpdateImageSize(imFrame.size());

    // Take the input video image, and convert it into the tracker's keyframe struct
    // This does things like generate the image pyramid and find FAST corners
    mCurrentKF->mMeasurements.clear();
    mCurrentKF->MakeKeyFrame(imFrame,imFrameD,mCamera.get());

    initNewFrame();
    if(!trackMap()) {
        // If there is no map, try to make one.
        mMapMaker.InitFromRGBD(*mCurrentKF);
        mnKeyFrames = 1;
    }

    processGUIEvents();
};

// use rgb image, for RGB-D SLAM using backend
void Tracker::TrackFrame(CVD::Image<CVD::Rgb<CVD::byte> > &imFrameRGB, CVD::Image<uint16_t> &imFrameD, bool isBgr)
{
    mMessageForUser.str("");   // Wipe the user message clean
    cout << "Doing tracking process ..." << endl;

    CVD::Image<CVD::byte> imFrame;
    imFrame.resize(imFrameRGB.size());
    CVD::convert_image(imFrameRGB,imFrame);
    UpdateImageSize(imFrame.size());

    // Take the input video image, and convert it into the tracker's keyframe struct
    // This does things like generate the image pyramid and find FAST corners
    mCurrentKF->mMeasurements.clear();
    mCurrentKF->MakeKeyFrame(imFrame,imFrameD,mCamera.get());
    mCurrentKF->rgbIsBgr_ = isBgr;

    // Add depth and rgb image to the keyframe
    mCurrentKF->rgbImage.resize(imFrameRGB.size());
    mCurrentKF->depthImage.resize(imFrameD.size());
    copy(imFrameRGB, mCurrentKF->rgbImage);
    copy(imFrameD, mCurrentKF->depthImage);

    initNewFrame();
    if(!trackMapDual()) {
        // If there is no map, try to make one.
        TooN::SE3<> IniPose = TooN::SE3<>();
        IniPose.get_translation()[1] = 0.5;
        TooN::Matrix<3> datam = TooN::Data(0, -1.0, 0,//Rww1, because the roll and pitch angles are in
                              0, 0, -1.0, // a world frame which pointing downward.
                              1.0, 0, 0);
        IniPose.get_rotation() = datam;
        mMapMaker.InitFromRGBD(*mCurrentKF, IniPose);
        mse3CamFromWorld = mMap.vpKeyFrames[0]->se3CfromW;
        mnKeyFrames = 1;
    }
};

// use multiple rgb images, for RGB-D SLAM using multi-kinect with a backend for PGO
void Tracker::TrackFrame(std::vector<CVD::Image<CVD::Rgb<CVD::byte> > > &imFrameRGB,
                         std::vector<CVD::Image<uint16_t> > &imFrameD,
                         std::vector<int> adcamIndex, bool isBgr)
{
    cout << "Dual images received" <<"\n";
    mUsingDualImg = true;
    assert((imFrameRGB.size() == imFrameD.size())
           && (imFrameRGB.size() == (adcamIndex.size() + 1))
           && (imFrameRGB.size() > 0));
    mMessageForUser.str("");   // Wipe the user message clean

    ros::Time timevobegin=ros::Time::now();
    if (trackerlog.is_open()){
        trackerlog << "frame time: " << timevobegin.toSec() << " ";
    }

    // Add the main camera data
    CVD::Image<CVD::byte> imFrame;
    imFrame.resize(imFrameRGB[0].size());
    CVD::convert_image(imFrameRGB[0],imFrame);
    UpdateImageSize(imFrame.size());
    // Take the input video image, and convert it into the tracker's keyframe struct
    // This does things like generate the image pyramid and find FAST corners
    mCurrentKF->mMeasurements.clear();
    mCurrentKF->MakeKeyFrame(imFrame,imFrameD[0],mCamera.get());
    mCurrentKF->rgbIsBgr_ = isBgr;
    // Add depth and rgb image to the keyframe
    mCurrentKF->rgbImage.resize(imFrameRGB[0].size());
    mCurrentKF->depthImage.resize(imFrameD[0].size());
    mCurrentKF->nSourceCamera = 0;
    copy(imFrameRGB[0], mCurrentKF->rgbImage);
    copy(imFrameD[0], mCurrentKF->depthImage);

    initNewFrame();

    // And data from other cameras. Correctly correspond the camera index
    for (int i = 0; i < AddCamNumber; i ++)
        mCurrentKFsec[i]->bNewsec = false;
    ActiveAdCamIndex = adcamIndex; // direct copy
    for (int i = 0; i < adcamIndex.size(); i ++) {
        CVD::Image<CVD::byte> imFrame;
        imFrame.resize(imFrameRGB[i+1].size());
        CVD::convert_image(imFrameRGB[i+1],imFrame);

        // Take the input video image, and convert it into the tracker's keyframe struct
        // This does things like generate the image pyramid and find FAST corners
        mCurrentKFsec[adcamIndex[i]]->mMeasurements.clear();
        mCurrentKFsec[adcamIndex[i]]->MakeKeyFrame(imFrame,imFrameD[i + 1],mCameraSec[adcamIndex[i]].get());
        mCurrentKFsec[adcamIndex[i]]->rgbIsBgr_ = isBgr;
        mCurrentKFsec[adcamIndex[i]]->nSourceCamera = adcamIndex[i] + 1; // mCameraSec begin with 0, while nSourceCamera begin with 1!

        // Add depth and rgb image to the keyframe
        mCurrentKFsec[adcamIndex[i]]->rgbImage.resize(imFrameRGB[i + 1].size());
        mCurrentKFsec[adcamIndex[i]]->depthImage.resize(imFrameD[i + 1].size());
        copy(imFrameRGB[i + 1], mCurrentKFsec[adcamIndex[i]]->rgbImage);
        copy(imFrameD[i + 1], mCurrentKFsec[adcamIndex[i]]->depthImage);

        initNewFrame_sec(adcamIndex[i]);
        mCurrentKFsec[adcamIndex[i]]->bNewsec = true;
    }

    if(!trackMapDual()) {
        // If there is no map, try to make one.
        TooN::SE3<> IniPose = TooN::SE3<>();
        IniPose.get_translation()[1] = 0.5;
        TooN::Matrix<3> datam = TooN::Data(0, -1.0, 0,//Rww1, because the roll and pitch angles are in
                              0, 0, -1.0, // a world frame which pointing downward.
                              1.0, 0, 0);
        IniPose.get_rotation() = datam;
        if ((adcamIndex.size() == AddCamNumber) && mMapMaker.InitFromRGBD(*mCurrentKF, mCurrentKFsec, IniPose)){
            mnKeyFrames = 1;
            mse3CamFromWorld = mMap.vpKeyFrames[0]->se3CfromW;
            for (int i = 0; i < AddCamNumber; i ++){
                mse3CamFromWorldsec[i] = mse3Cam1FromCam2[i].inverse()*mse3CamFromWorld;
                mnKeyFramessec[i] = 1;
            }
        }
    }

    ros::Time timevoend = ros::Time::now();
    ros::Duration timevo = timevoend - timevobegin;
    timecost_vo = timevo.toSec();

    if (trackerlog.is_open()){

        trackerlog << "vo time: "
                   << timecost_vo <<  " "
                   << "\n";
    }

    processGUIEvents();
};

// Tracks a frame obtained through stereo matching
void Tracker::TrackFrame(CVD::Image<CVD::byte> &imFrame, const sensor_msgs::PointCloud& points)
{
    mMessageForUser.str("");   // Wipe the user message clean

    UpdateImageSize(imFrame.size());

    // Take the input video image, and convert it into the tracker's keyframe struct
    mCurrentKF->mMeasurements.clear();
    mCurrentKF->MakeKeyFrame(imFrame, points, mCamera.get());

    initNewFrame();
    if(!trackMap()) {
        // If there is no map, try to make one.
        mMapMaker.InitFromRGBD(*mCurrentKF);
        mnKeyFrames = 1;
    }

    processGUIEvents();
};

// Performs initial processing for a new frame
void Tracker::initNewFrame() {
    // Update the small images for the rotation estimator
    static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
    static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
    mbUseSBIInit = *gvnUseSBI;
    if(!mpSBIThisFrame)
    {
        mpSBIThisFrame = new SmallBlurryImage(*mCurrentKF, *gvdSBIBlur);
        mpSBILastFrame = new SmallBlurryImage(*mCurrentKF, *gvdSBIBlur);
    }
    else
    {
        delete  mpSBILastFrame;
        mpSBILastFrame = mpSBIThisFrame;
        mpSBIThisFrame = new SmallBlurryImage(*mCurrentKF, *gvdSBIBlur);
    }

    // From now on we only use the keyframe struct!
    mnFrame++;
}
void Tracker::initNewFrame_sec(int camIndex) {
    // Update the small images for the rotation estimator
    static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
    if (!mpSBIThisFramesec[camIndex])
    {
        mpSBIThisFramesec[camIndex] = new SmallBlurryImage(*mCurrentKFsec[camIndex], *gvdSBIBlur);
        mpSBILastFramesec[camIndex] = new SmallBlurryImage(*mCurrentKFsec[camIndex], *gvdSBIBlur);
    }
    else
    {
        delete  mpSBILastFramesec[camIndex];
        mpSBILastFramesec[camIndex] = mpSBIThisFramesec[camIndex];
        mpSBIThisFramesec[camIndex] = new SmallBlurryImage(*mCurrentKFsec[camIndex], *gvdSBIBlur);
    }

    // From now on we only use the keyframe struct!
    mnFramesec[camIndex]++;
}
// If there is a map, try to track the map. Otherwise return false
bool Tracker::trackMap() {

    // Decide what to do -
    if(!mMap.IsGood())
        return false;

    if(mnLostFrames < 3 //)  // .. but only if we're not lost!
            || ((mnLostFrames < 5) && pose_ekf_get))// yang, try use EKF prediction
    {
        if(mbUseSBIInit && (mnLostFrames < 3))
            CalcSBIRotation();
        //                ApplyMotionModel();       //  yang, now use motion model updated with EKF info
        if (pose_ekf_get && mnLostFrames >= 3)
            ApplyMotionModel_EKF();     //  TODO: use pose prediction from EKF only when the filter is trustable, not at the beginning
        else
            ApplyMotionModel();
        TrackMap();               //  These three lines do the main tracking work.
        cout << "kf pose: " << mCurrentKF->se3CfromW << endl;

        // implement the scale factor from last image before add a new keyframe.
        // this would effect both the pose estimation of the keyframe and those new map points
        //                mCurrentKF->se3CfromW.get_translation() = mCurrentKF->se3CfromW.get_translation() / scale_ekf;

        UpdateMotionModel();      //

        MotionCheck();            // Check for tracking failure

        AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.

        { // Provide some feedback for the user:
            mMessageForUser << "Tracking Map, quality ";
            if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
            if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
            if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
            mMessageForUser << " Found:";
            for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
            //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
            mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
        }


        static gvar3<int> minInterval("Tracker.MinKeyFrameInterval", 20, SILENT);
        // Heuristics to check if a key-frame should be added to the map:
        // force to add kfs when both cam kfs avilable
        if(mTrackingQuality == GOOD &&
                mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
                (mnFrame - mnLastKeyFrameDropped) > *minInterval  &&
                mMapMaker.QueueSize() < 3)
        {
            mMessageForUser << " Adding key-frame.";
            //			assert(mCurrentKF.aLevels[0].vCandidates.size() > 0);
            // if use dual img, add them both
            AddNewKeyFrame();
        };
    }
    else  // what if there is a map, but tracking has been lost?
    {
        mMessageForUser << "** Attempting recovery **.";
        if(AttemptRecovery())
        {
            TrackMap();
            AssessTrackingQuality();
        }
    }

    tracking_map = true;

    return true;
}
// dual img tracking. If there is a map, try to track the map. Otherwise return false
bool Tracker::trackMapDual() {

    // Decide what to do -
    if(!mMap.IsGood())
        return false;

    // for dual mono slam,
    // 1. only both camera lose tracking, then we assume it lose tracking
    // 2. feature tracking should be done on corresponding dual image seperately, i.e. assuming features from diff oriented camera
    // do not share common appearance.
    if(mnLostFrames < 3 //)  // .. but only if we're not lost!
            || ((mnLostFrames < 5) && pose_ekf_get))// yang, try use EKF prediction
    {
        // yang, sbi from dual image should be orgernised to calc rotation
        if(mbUseSBIInit && (mnLostFrames < 3))
            CalcSBIRotation();
        //                ApplyMotionModel();       //  yang, now use motion model updated with EKF info
        if (pose_ekf_get && mnLostFrames >= 3)
            ApplyMotionModel_EKF();     //  TODO: use pose prediction from EKF only when the filter is trustable, not at the beginning
        else
            ApplyMotionModel();

        ros::Time timetrack_b = ros::Time::now();

        TrackMap();               //  These three lines do the main tracking work.

        ros::Time timetrackend = ros::Time::now();
        ros::Duration timetrack = timetrackend - timetrack_b;
        if (trackerlog.is_open())
            trackerlog << "trackmap time: " << timetrack.toSec() << " ";

        // implement the scale factor from last image before add a new keyframe.
        // this would effect both the pose estimation of the keyframe and those new map points
        //                mCurrentKF->se3CfromW.get_translation() = mCurrentKF->se3CfromW.get_translation() / scale_ekf;

        UpdateMotionModel();      //

        MotionCheck();            // Check for tracking failure

        AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.

        { // Provide some feedback for the user:
            mMessageForUser << "Tracking Map, quality ";
            if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
            if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
            if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
            mMessageForUser << " Found:";
            for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
            //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
            mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size()
                            << "+" << mMap.vpKeyFramessec[0].size()<< "KF" ;
        }

        static gvar3<int> minInterval("Tracker.MinKeyFrameInterval", 20, SILENT);
        // Heuristics to check if a key-frame should be added to the map:
        // TODO: also consider the second image?
        if(mTrackingQuality == GOOD &&
                mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
                (mnFrame - mnLastKeyFrameDropped) > *minInterval  &&
                mMapMaker.QueueSize() < 3)
        {
            mMessageForUser << " Adding key-frame.";
            //			assert(mCurrentKF.aLevels[0].vCandidates.size() > 0);
            AddNewKeyFrame();
        }
        else if((mTrackingQuality == GOOD || mTrackingQuality == DODGY) && // too far away without a new keyframe, deem to fail pose tracking
                mMapMaker.NeedErgentKeyFrame(mCurrentKF) &&
                (mnFrame - mnLastKeyFrameDropped) > *minInterval  &&
                mMapMaker.QueueSize() < 3)
        {
            mMessageForUser << " Adding Ergent key-frame.";
            //			assert(mCurrentKF.aLevels[0].vCandidates.size() > 0);
            AddNewKeyFrame();
        };
        /// TODO: such Ergent keyframe should be handled in bundle adjustment and the backend

        /// TODO: record the most recent good kf for tracking recovery
//        if (mTrackingQuality == GOOD){
//            *mGoodKFtoTrack = *mCurrentKF;
//            for (int cn = 0; cn < AddCamNumber; cn ++)
//                mGoodKFtoTracksec[cn] = mCurrentKFsec[cn];
//        }
    }
    else  // what if there is a map, but tracking has been lost?
    {
        mMessageForUser << "** Attempting recovery **.";
        //TODO: this should also use dual img
        ros::Time timerecoverb = ros::Time::now();

        bool mbrecovered = AttemptRecovery();

        ros::Time timerecoverend = ros::Time::now();
        ros::Duration timerecover = timerecoverend - timerecoverb;
        if (trackerlog.is_open())
            trackerlog << "recover time: " << timerecover.toSec() << "\n";

        if(mbrecovered)
        {
            mnFailureFrames = 0;
            TrackMap();
            AssessTrackingQuality();

            { // Provide some feedback for the user:
                mMessageForUser << "Tracking Map, quality ";
                if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
                if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
                if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
                mMessageForUser << " Found:";
                for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
                //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
                mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF" ;
            }
            cout << "Tracked position after recovery: " << endl
                    << mCurrentKF->se3CfromW.inverse().get_translation() << endl;
        }
        else { // handle complete failures: restart the SLAM by adding a new keyframe, old kfs should not be used
            // for localization any more until a loop closing is done. A new local map is formed.
            // such local map could be formed in a vector of maps.
            // to simplify this problem. only one local map is retained. renew the local map if tracking fail again.
            ApplyMotionModel();
            mnFailureFrames ++;
            trackerlogdebug << "apply motion model after failed to recover!" << endl;

            if (mnFailureFrames > 2){
                mnFailureFrames = 0;

                // or we can say re-initialize the map: just more complex
                // in the backend, links between this node to the previous ones should be handled
                TooN::SE3<> IniPose = mse3CamFromWorld;
                if (mMapMaker.ReInitFromRGBD(*mCurrentKF, mCurrentKFsec, IniPose)){
                    mbOldMaplocked = true;
                    trackerlogdebug << "map re-initialized!" << endl;
                }
            }
        }
    }

    tracking_map = true;

    return true;
}

// Processes all queued GUI events
void Tracker::processGUIEvents() {
    while(!mvQueuedCommands.empty())
    {
        GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
        mvQueuedCommands.erase(mvQueuedCommands.begin());
    }
}

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
    cout << "Try to recorver..." << endl;
    bool bRelocGood = false;
    bool bRelocGoodsec = false;
    int nAdCamGoodnum = 0;
    TooN::SE3<> mbestpose;
    SE3<> se3Best;
    static gvar3<int> gvnUsePnPrecovery("Tracker.usePnPrecover", 1, SILENT);
    static gvar3<double> gvnMinInliers("Tracker.MinInliers", 0.5, SILENT);
    double minInliers = *gvnMinInliers;

    ros::Time trecoverb = ros::Time::now();
    trackerlogdebug << trecoverb.toSec() << " Pose before Recovering: " << endl;
    trackerlogdebug << mse3CamFromWorld.inverse().get_translation() << endl;

    /// use multi image to relocalize the system
    /// use new reloc. method: RANSAC+PnP w.r.t the lated well-tracked frame.
    if (*gvnUsePnPrecovery){
        // read access: unique lock
        boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);
//        boost::shared_ptr<KeyFrame> pClosest = mMapMaker.ClosestKeyFrame(mCurrentKF);
        boost::shared_ptr<KeyFrame> pClosest = mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1];
        mGoodKFtoTrack = pClosest;
        lock.unlock();

        bRelocGood = AttemptRecovery(mGoodKFtoTrack, mCurrentKF, mbestpose, minInliers);
        if (!bRelocGood && mUsingDualImg)
            for (int i = 0; i < AddCamNumber; i ++){
                if (bRelocGoodsec)
                    break;
                lock.lock();
                pClosest = mMap.vpKeyFramessec[i][mMap.vpKeyFramessec[i].size() - 1];
                mGoodKFtoTracksec[i] = pClosest;
                lock.unlock();

                if (mCurrentKFsec[i]->bNewsec)
                    bRelocGoodsec = AttemptRecovery(mGoodKFtoTracksec[i], mCurrentKFsec[i], mbestpose, minInliers);
                nAdCamGoodnum = i;
            }
        if(!bRelocGood && !bRelocGoodsec){
            cout << "Recovering failed using PnP method." << endl;
            trackerlogdebug << "Recovering failed using PnP method." << endl;
            return false;
        }

        if (bRelocGood)
            se3Best = mbestpose.inverse() * mGoodKFtoTrack->se3CfromW;
        else if (bRelocGoodsec){
            se3Best = mbestpose.inverse() * mGoodKFtoTracksec[nAdCamGoodnum]->se3CfromW ;
            se3Best = mse3Cam1FromCam2[nAdCamGoodnum] * se3Best;
        }
        mse3CamFromWorld = se3Best; mse3StartPos = se3Best;
        mse3CamFromWorldPub = se3Best; mse3StartPos = se3Best;
        for (int i = 0; i < AddCamNumber; i ++)
            mse3CamFromWorldsec[i] = mse3Cam1FromCam2[i].inverse()*mse3CamFromWorld;
        mv6CameraVelocity = Zeros;
        mbJustRecoveredSoUseCoarse = true;
        cout << "Recovering seems to be success using PnP method." << endl;
        trackerlogdebug << "Recovering seems to be success using PnP method." << endl;
        trackerlogdebug << mse3CamFromWorld.inverse().get_translation() << endl;
        return true;
    }

    /// If failed, then use the original method of PTAM to reloc.
    if (!bRelocGood && !bRelocGoodsec){
        bRelocGood = mRelocaliser.AttemptRecovery(*mCurrentKF);
        /// using only one other kf for reloc, after the main camera kf failed
        if (!bRelocGood && mUsingDualImg)
            for (int i = 0; i < AddCamNumber; i ++){
                if (bRelocGoodsec)
                    break;
                if (mCurrentKFsec[i]->bNewsec)
                    bRelocGoodsec = mRelocaliser.AttemptRecovery(*mCurrentKFsec[i]);
                nAdCamGoodnum = i;
            }
        if(!bRelocGood && !bRelocGoodsec){
            cout << "Recovering failed using PTAM method." << endl;
            trackerlogdebug << "Recovering failed using PTAM method." << endl;
            return false;
        }
    }

    se3Best = mRelocaliser.BestPose();
    if (bRelocGoodsec)
        se3Best = mse3Cam1FromCam2[nAdCamGoodnum] * se3Best;
    mse3CamFromWorld = se3Best; mse3StartPos = se3Best;
    mse3CamFromWorldPub = se3Best; mse3StartPos = se3Best;
    for (int i = 0; i < AddCamNumber; i ++)
        mse3CamFromWorldsec[i] = mse3Cam1FromCam2[i].inverse()*mse3CamFromWorld;
    mv6CameraVelocity = Zeros;
    mbJustRecoveredSoUseCoarse = true;
    cout << "Recovering seems to be success using PTAM method." << endl;
    trackerlogdebug << "Recovering seems to be success using PTAM method." << endl;
    return true;
}

bool Tracker::AttemptRecovery(boost::shared_ptr<KeyFrame> goodkf, boost::shared_ptr<KeyFrame> kf, TooN::SE3<> &mse3Best, double minInliers)
{
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);
    goodkf->finalizeKeyframeGoodkf();
    boost::shared_ptr<KeyFrame> goodkftrack(new KeyFrame);
    *goodkftrack = *goodkf;
    lock.unlock();

    cout << "goodkf id: " << goodkf->id << endl;
    kf->finalizeKeyframekpts();
    Sophus::SE3d relPoseAB;

    if (mMapMaker.relocaliseRegister(goodkftrack, kf, relPoseAB, minInliers)){
        mse3Best = cs_geom::toToonSE3(relPoseAB);
        return true;
    }
    else
        return false;
}

// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
    Command c;
    c.sCommand = sCommand;
    c.sParams = sParams;
    ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
    if(sCommand=="Reset")
    {
        Reset();
        return;
    }

    // KeyPress commands are issued by GLWindow
    if(sCommand=="KeyPress")
    {
        if(sParams == "Space")
        {
            mbUserPressedSpacebar = true;
        }
        else if(sParams == "r")
        {
            Reset();
        }
        else if(sParams == "q" || sParams == "Escape")
        {
            GUI.ParseLine("quit");
        }
        return;
    }
    if((sCommand=="PokeTracker"))
    {
        mbUserPressedSpacebar = true;
        return;
    }


    cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
    exit(1);
}; 

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
void Tracker::TrackForInitialMap()
{
    // MiniPatch tracking threshhold.
    static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
    MiniPatch::mnMaxSSD = *gvnMaxSSD;

    static int trailFrames = 0;

    // What stage of initial tracking are we at?
    if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
    {
        if(true) // mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
        {
            mbUserPressedSpacebar = false;
            if (!use_circle_ini){
                TrailTracking_Start();
                mnInitialStage = TRAIL_TRACKING_STARTED;
            }
            else if (circle_pose_get){
                if (se3CfromW.inverse().get_translation()[2] > 0.6){//young, set the initialize minimal height
                    mCurrentKF->se3CfromW = se3CfromW;
                    TrailTracking_Start();
                    mnInitialStage = TRAIL_TRACKING_STARTED;
                    circle_pose_get = false;

                    //log the pose info of the camera used for initialization
//                    trackerlog << time_stamp_circle << " " << time_stamp_now << " " << se3CfromW.get_translation()[0] << " " <<
//                                  se3CfromW.get_translation()[1] << " " << se3CfromW.get_translation()[2] << std::endl;

                }
                else
                    mMessageForUser << "Altitude too low: "<<se3CfromW.inverse().get_translation()[2];
            }
            else{
                mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
                mMessageForUser << "Circle pose NOT get.";
            }
            trailFrames = 0;
        }
        else
            mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map." << endl;
        return;
    }

    if(mnInitialStage == TRAIL_TRACKING_STARTED)
    {
        int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
        trailFrames++;
        if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
        {
            mMessageForUser << "Trail not good." <<endl;
            Reset();
            return;
        }

        // If the user pressed spacebar here, use trails to run stereo and make the intial map..
        if(trailFrames > 10) // mbUserPressedSpacebar)f
        {
            mbUserPressedSpacebar = false;
            vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
            for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
                vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos,
                                                            i->irCurrentPos));

            if (!use_circle_ini){
                if (mMapMaker.InitFromStereo(mFirstKF, *mCurrentKF, vMatches, mse3CamFromWorld, false, 0, 0))  // This will take some time!
                {
                    mnInitialStage = TRAIL_TRACKING_COMPLETE;
                    mnKeyFrames = 2;
                }
            }
            else if (circle_pose_get){
                static int circle_get_thresh = 3;//filter outlier
                static int circle_pose_count = 0;
                mCurrentKF->se3CfromW = se3CfromW;
                if (mMapMaker.InitFromStereo(mFirstKF, *mCurrentKF, vMatches, mse3CamFromWorld, true, circle_get_thresh -1, circle_pose_count))
                {
                    circle_pose_count ++;
                    if (circle_pose_count >= circle_get_thresh){
                        mnInitialStage = TRAIL_TRACKING_COMPLETE;
                        ini_circle_finished = true;
                        mnKeyFrames = 2;
                        circle_pose_count = 0;
                    }

                    //log the pose info of the camera used for initialization
//                    trackerlog << time_stamp_circle << " " << time_stamp_now << " " << se3CfromW.get_translation()[0] << " " <<
//                                  se3CfromW.get_translation()[1] << " " << se3CfromW.get_translation()[2] << std::endl;
                }
                else
                    circle_pose_count = 0;
            }
            else
                mnInitialStage = TRAIL_TRACKING_STARTED;
        }
        else
            mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init." << endl;

        circle_pose_get = false;
    }
}


// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start()
{
    mCurrentKF->MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
    mFirstKF = *mCurrentKF;
    vector<pair<double,ImageRef> > vCornersAndSTScores;
    for(unsigned int i=0; i<mCurrentKF->aLevels[0].vCandidates.size(); i++)  // Copy candidates into a trivially sortable vector
    {                                                                     // so that we can choose the image corners with max ST score
        Candidate &c = mCurrentKF->aLevels[0].vCandidates[i];
        if(!mCurrentKF->aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
            continue;
        vCornersAndSTScores.push_back(pair<double,ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
    };
    sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
    int nToAdd = GV2.GetInt("MaxInitialTrails", 1000, SILENT);
    for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
    {
        if(!mCurrentKF->aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
            continue;
        Trail t;
        t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF->aLevels[0].im);
        t.irInitialPos = vCornersAndSTScores[i].second;
        t.irCurrentPos = t.irInitialPos;
        mlTrails.push_back(t);
        nToAdd--;
    }

    mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.


}

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int Tracker::TrailTracking_Advance()
{
    int nGoodTrails = 0;

    MiniPatch BackwardsPatch;
    Level &lCurrentFrame = mCurrentKF->aLevels[0];
    Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

    for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
        list<Trail>::iterator next = i; next++;

        Trail &trail = *i;
        ImageRef irStart = trail.irCurrentPos;
        ImageRef irEnd = irStart;
        bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
        if(bFound)
        {
            // Also find backwards in a married-matches check
            BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
            ImageRef irBackWardsFound = irEnd;
            bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
            if((irBackWardsFound - irStart).mag_squared() > 2)
                bFound = false;

            trail.irCurrentPos = irEnd;
            nGoodTrails++;
        }
        if(!bFound) // Erase from list of trails if not found this frame.
        {
            mlTrails.erase(i);
        }
        i = next;
    }

    mPreviousFrameKF = *mCurrentKF;
    return nGoodTrails;
}

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// * we should only project map points with the same source camera to the correspondng camera img
// Then it tries to find some points of the PVS in the image;
// * feature searching is done on individual dual images
// * if we use a homoginious map, we should identify the source camera of the map points.
// Then it updates camera pose according to any points found.
// * points from the slavery image are also integrated in the optimisation
// * to achieve an optimised pose estimation of the master camera
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
/// Using multi-kinect: use no more than two cameras for coarse tracking as before;
///
void Tracker::TrackMap()
{
    // Some accounting which will be used for tracking quality assessment:
    // aslo should be counted on a dual-image bases.
    for(int i=0; i<LEVELS; i++)
        manMeasAttempted[i] = manMeasFound[i] = 0;

    static gvar3<int> gvnTrackCam2camError("Tracker.TrackCam2camError", 0, SILENT);
    bool TrackCam2camError = false;// *gvnTrackCam2camError;
    // parameters used for debug
    bool use_seccam_track = false;// use only second camera for tracking, only for test
    bool poseupdate_cam2 = false;// use cam2 as main pose update cam?
    bool use_seccamonly = false;

    // The Potentially-Visible-Set (PVS) is split into pyramid levels.
    vector<boost::shared_ptr<MapPoint> > avPVS[LEVELS];
    for(int i=0; i<LEVELS; i++)
        avPVS[i].reserve(500);
    vector<boost::shared_ptr<MapPoint> > avPVSsec[AddCamNumber][LEVELS];
    for(int i=0; i<LEVELS; i++)
        for (int j = 0; j < AddCamNumber; j ++)
        avPVSsec[j][i].reserve(500);

//    debugmarkLoopDetected = false;
    // read access: shared lock
    boost::shared_lock< boost::shared_mutex > lock(mMap.mutex);

    // For thread safe, update motion model which caused by local map update from pgo, here
    if (mMapMaker.needMotionModelUpdate){
        MotionModelUpdateByGMap();
        if (mMapMaker.debugmarkLoopDetected){
            debugmarkLoopDetected = true;
            mMapMaker.debugmarkLoopDetected = false;
        }
        mMapMaker.needMotionModelUpdate = false;
    }

    SE3<> posesecCamFromWorld[AddCamNumber];

    for (int i = 0; i < AddCamNumber; i ++)
        posesecCamFromWorld[i] = mse3Cam1FromCam2[i].inverse() * mse3CamFromWorld;
    if (mUsingDualImg && use_seccamonly){
        posesecCamFromWorld[0] = mse3CamFromWorldsec[0];
        mse3CamFromWorld = mse3Cam1FromCam2[0] * posesecCamFromWorld[0];
    }

    // For dual image, PVS should be searched on individual images.
    // For all points in the map..
    /// search all mp in all cameras now
    /// when there's no overlapping FOV, each point has a single trakerdata,
    /// otherwise, multiple trackerdata can be coded, or a more straightforward way is
    /// only track the point in one camera
    for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
        if (mMap.vpPoints[i]->mblocked)
            continue;

        boost::shared_ptr<MapPoint> p= mMap.vpPoints[i];
        TrackerData &TData = p->TData;

        if (!p->nSourceCamera){ // from cam 0
            if (p->sourceKfIDtransfered && !p->refreshed){
                p->v3Center_NC = unproject(mCamera->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos));
                p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos + vec(ImageRef(p->pPatchSourceKF.lock()->mMeasurements[p].nLevel,0))));
                p->v3OneDownFromCenter_NC  = unproject(mCamera->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos + vec(ImageRef(0,p->pPatchSourceKF.lock()->mMeasurements[p].nLevel))));
                normalize(p->v3Center_NC);
                normalize(p->v3OneDownFromCenter_NC);
                normalize(p->v3OneRightFromCenter_NC);
                p->RefreshPixelVectors();

                p->refreshed = true;
            }
        }
        else {
            int adcamIndex = p->nSourceCamera - 1;

            if (p->sourceKfIDtransfered && !p->refreshed){
                p->v3Center_NC = unproject(mCameraSec[adcamIndex]->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos));
                p->v3OneRightFromCenter_NC = unproject(mCameraSec[adcamIndex]->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos + vec(ImageRef(p->pPatchSourceKF.lock()->mMeasurements[p].nLevel,0))));
                p->v3OneDownFromCenter_NC  = unproject(mCameraSec[adcamIndex]->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos + vec(ImageRef(0,p->pPatchSourceKF.lock()->mMeasurements[p].nLevel))));
                normalize(p->v3Center_NC);
                normalize(p->v3OneDownFromCenter_NC);
                normalize(p->v3OneRightFromCenter_NC);
                p->RefreshPixelVectors();

                p->refreshed = true;
            }
        }

        /// each mp should be checked for each camera
        /// First, for the master camera
        // Project according to current view, and if it's not in the image, skip.
        TData.Project(p->v3WorldPos, mse3CamFromWorld, mCamera.get());
        if (TData.bInImage)
        {
            // Hack by Jonathan Klimesch: Need to see a point from roughly the same angle.
            double cosAngle;
            {
                Vector<3> z;
                z[0] = z[1] = 0.0; z[2] = 1.0;
                boost::shared_ptr<KeyFrame> kf = p->pPatchSourceKF.lock();
                if(kf.get() == NULL)
                    continue;// done

                //            cosAngle = (mCurrentKF->se3CfromW.get_rotation()*z)*(kf->se3CfromW.get_rotation()*z);
                cosAngle = (mse3CamFromWorld.get_rotation()*z)*(kf->se3CfromW.get_rotation()*z);
            }
            if (cosAngle < 0) { // |angle| > 90°
                continue; // done
            }

            // Calculate camera projection derivatives of this point.
            TData.GetDerivsUnsafe(mCamera.get());

            // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
            TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(*p, mse3CamFromWorld, TData.m2CamDerivs);
            if(TData.nSearchLevel == -1) {
                continue;   // done. a negative search pyramid level indicates an inappropriate warp for this view, so skip.
            }
            // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
            TData.bSearched = false;
            TData.bFound = false;
            p->nFoundCamera = 0;
            avPVS[TData.nSearchLevel].push_back(p);

            continue; // done
        }
        else if (mUsingDualImg)
        {
            for (int cn = 0; cn < AddCamNumber; cn ++)
            {
                // Project according to current view, and if it's not in the image, skip.
                TData.Project(p->v3WorldPos, mse3CamFromWorldsec[cn], mCameraSec[cn].get());
                if (TData.bInImage){
                    // Hack by Jonathan Klimesch: Need to see a point from roughly the same angle.
                    double cosAngle;
                    {
                        Vector<3> z;
                        z[0] = z[1] = 0.0; z[2] = 1.0;
                        boost::shared_ptr<KeyFrame> kf = p->pPatchSourceKF.lock();
                        if(kf.get() == NULL)
                            continue;

                        //            cosAngle = (mCurrentKF->se3CfromW.get_rotation()*z)*(kf->se3CfromW.get_rotation()*z);
                        cosAngle = (mse3CamFromWorldsec[cn].get_rotation()*z)*(kf->se3CfromW.get_rotation()*z);
                    }
                    if (cosAngle < 0) { // |angle| > 90°
                        continue;
                    }

                    // Calculate camera projection derivatives of this point.
                    TData.GetDerivsUnsafe(mCameraSec[cn].get());

                    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
                    TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(*p, mse3CamFromWorldsec[cn], TData.m2CamDerivs);
                    if(TData.nSearchLevel == -1) {
                        continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
                    }
                    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
                    TData.bSearched = false;
                    TData.bFound = false;
                    p->nFoundCamera = cn + 1;
                    avPVSsec[cn][TData.nSearchLevel].push_back(p);
                    break;
                }
            }
        }
        if (false)//else if (mUsingDualImg)// on the second image
        {
            boost::shared_ptr<MapPoint> p= mMap.vpPoints[i];
            TrackerData &TData = p->TData;
            int adcamIndex = p->nSourceCamera - 1;

            if (p->sourceKfIDtransfered && !p->refreshed){
                p->v3Center_NC = unproject(mCameraSec[adcamIndex]->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos));
                p->v3OneRightFromCenter_NC = unproject(mCameraSec[adcamIndex]->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos + vec(ImageRef(p->pPatchSourceKF.lock()->mMeasurements[p].nLevel,0))));
                p->v3OneDownFromCenter_NC  = unproject(mCameraSec[adcamIndex]->UnProject(p->pPatchSourceKF.lock()->mMeasurements[p].v2RootPos + vec(ImageRef(0,p->pPatchSourceKF.lock()->mMeasurements[p].nLevel))));
                normalize(p->v3Center_NC);
                normalize(p->v3OneDownFromCenter_NC);
                normalize(p->v3OneRightFromCenter_NC);
                p->RefreshPixelVectors();

                p->refreshed = true;
            }

            // Project according to current view, and if it's not in the image, skip.
            TData.Project(p->v3WorldPos, posesecCamFromWorld[adcamIndex], mCameraSec[adcamIndex].get());
            if(!TData.bInImage) {
                continue;
            }

            // Hack by Jonathan Klimesch: Need to see a point from roughly the same angle.
            double cosAngle;
            {
                Vector<3> z;
                z[0] = z[1] = 0.0; z[2] = 1.0;
                boost::shared_ptr<KeyFrame> kf = p->pPatchSourceKF.lock();
                if(kf.get() == NULL)
                    continue;

                cosAngle = (posesecCamFromWorld[adcamIndex].get_rotation()*z)*(kf->se3CfromW.get_rotation()*z);
            }
            if (cosAngle < 0) { // |angle| > 90°
                continue;
            }

            // Calculate camera projection derivatives of this point.
            TData.GetDerivsUnsafe(mCameraSec[adcamIndex].get());

            // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
            TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(*p, posesecCamFromWorld[adcamIndex], TData.m2CamDerivs);
            if(TData.nSearchLevel == -1) {
                continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
            }
            // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
            TData.bSearched = false;
            TData.bFound = false;
            p->nFoundCamera = adcamIndex + 1;
            avPVSsec[adcamIndex][TData.nSearchLevel].push_back(p);
        }
    }

    lock.unlock(); // problem: ownership of TData unclear

    // Next: A large degree of faffing about and deciding which points are going to be measured!
    // First, randomly shuffle the individual levels of the PVS.
    for(int i=0; i<LEVELS; i++)
    {
        random_shuffle(avPVS[i].begin(), avPVS[i].end());
        if (mUsingDualImg)
            for (int j = 0; j < AddCamNumber; j ++)
            random_shuffle(avPVSsec[j][i].begin(), avPVSsec[j][i].end());
    }

    // The next two data structs contain the list of points which will next
    // be searched for in the image, and then used in pose update.
    vector<boost::shared_ptr<MapPoint> > vNextToSearch;
    vector<boost::shared_ptr<MapPoint> > vIterationSet;
    // for the second img
    vector<boost::shared_ptr<MapPoint> > vNextToSearchsec[AddCamNumber];

    // Tunable parameters to do with the coarse tracking stage:
    static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
    static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
    static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 50, SILENT);       // Pixel search radius for coarse features
    static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
    static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
    static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.0, SILENT);  // Speed above which coarse stage is used.

    unsigned int nCoarseMax = *gvnCoarseMax;
    unsigned int nCoarseRange = *gvnCoarseRange;

    mbDidCoarse = false;

    // Set of heuristics to check if we should do a coarse tracking stage.
    bool bTryCoarse = true;
    if(*gvnCoarseDisabled ||
            mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
            nCoarseMax == 0)
        bTryCoarse = false;
    if(mbJustRecoveredSoUseCoarse)
    {
        bTryCoarse = true;
        nCoarseMax *=2;
        nCoarseRange *=2;
        mbJustRecoveredSoUseCoarse = false;
    };

//    bTryCoarse = false;//debug

//    if (bTryCoarse)
//        cout << "Cam pose before coarse: " << mse3CamFromWorld << endl;

    // If we do want to do a coarse stage, also check that there's enough high-level
    // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
    // with preference to LEVELS-1.
    // this should also based on dual image
    unsigned int nFound = 0;
    bool nPvsEnoughFirstImg = false;// master img already has enough pvs
    if(bTryCoarse && (avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size()) > *gvnCoarseMin )
    {
        // Now, fill the vNextToSearch struct with an appropriate number of
        // TrackerDatas corresponding to coarse map points! This depends on how many
        // there are in different pyramid levels compared to CoarseMin and CoarseMax.

        if(avPVS[LEVELS-1].size() <= nCoarseMax)
        { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
            vNextToSearch = avPVS[LEVELS-1];
            avPVS[LEVELS-1].clear();
        }
        else
        { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
            for(unsigned int i=0; i<nCoarseMax; i++){
                vNextToSearch.push_back(avPVS[LEVELS-1][i]);
            }
            avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
        }

        // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
        if(vNextToSearch.size() < nCoarseMax)
        {
            unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
            if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
            {
                // bug of original ptam fixed, should add all avPVS[LEVELS-2] to vNextToSearch
//                vNextToSearch = avPVS[LEVELS-2];
                for (unsigned int i=0; i<avPVS[LEVELS-2].size(); i++)
                    vNextToSearch.push_back(avPVS[LEVELS-2][i]);
                avPVS[LEVELS-2].clear();
            }
            else
            {
                for(unsigned int i=0; i<nMoreCoarseNeeded; i++){
                    vNextToSearch.push_back(avPVS[LEVELS-2][i]);
                }
                avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
            }
        }
        // Now go and attempt to find these points in the image!
        nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
        vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.

        if(nFound >= *gvnCoarseMin)
            nPvsEnoughFirstImg = true;
        // if only use one img, try coarse pose update now!
        if (!mUsingDualImg && nPvsEnoughFirstImg){
            mbDidCoarse = true;
            for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
            {
                if(iter != 0)
                { // Re-project the points on all but the first iteration.
                    for(unsigned int i=0; i<vIterationSet.size(); i++)
                        if(vIterationSet[i]->TData.bFound)
                            vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
                }
                for(unsigned int i=0; i<vIterationSet.size(); i++)
                    if(vIterationSet[i]->TData.bFound)
                        vIterationSet[i]->TData.CalcJacobian();
                double dOverrideSigma = 0.0;
                // Hack: force the MEstimator to be pretty brutal
                // with outliers beyond the fifth iteration.
                if(iter > 5)
                    dOverrideSigma = 1.0;

                // Calculate and apply the pose update...
                Vector<6> v6Update =
                        CalcPoseUpdate(vIterationSet, dOverrideSigma);
                mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
                for (int cn = 0; cn < AddCamNumber; cn ++)
                    posesecCamFromWorld[cn] = mse3Cam1FromCam2[cn].inverse() * mse3CamFromWorld;
            };
        }
        else{// do calculation with points in the second img been found later
        }
    } else {
        // not enough high-level PV map points
        //            std::cout << __PRETTY_FUNCTION__ << " not enough high-level PV map points for coarse search" << std::endl;
    }

    // TODO: find out the bug why cannot use sec cam for coarse tracking.
    // coarse step for the second img, no matter wether master img has enough pvs or not
    // if not, do alone, otherwise, add pvs to the pvss from the master img.
    /// only consider one of the additional cameras
    if (bTryCoarse && mUsingDualImg){
        if((avPVSsec[0][LEVELS-1].size() + avPVSsec[0][LEVELS-2].size()) > *gvnCoarseMin )
        {
            use_seccam_track = use_seccamonly;
            // Now, fill the vNextToSearch struct with an appropriate number of
            // TrackerDatas corresponding to coarse map points! This depends on how many
            // there are in different pyramid levels compared to CoarseMin and CoarseMax.

            if(avPVSsec[0][LEVELS-1].size() <= nCoarseMax)
            { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
                vNextToSearchsec[0] = avPVSsec[0][LEVELS-1];
                avPVSsec[0][LEVELS-1].clear();
            }
            else
            { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
                for(unsigned int i=0; i<nCoarseMax; i++)
                    vNextToSearchsec[0].push_back(avPVSsec[0][LEVELS-1][i]);
                avPVSsec[0][LEVELS-1].erase(avPVSsec[0][LEVELS-1].begin(), avPVSsec[0][LEVELS-1].begin() + nCoarseMax);
            }

            // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
            if(vNextToSearchsec[0].size() < nCoarseMax)
            {
                unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearchsec[0].size();
                if(avPVSsec[0][LEVELS-2].size() <= nMoreCoarseNeeded)
                {
                    // bug of original ptam fixed, should add all avPVS[LEVELS-2] to vNextToSearch
    //                vNextToSearchsec = avPVS[LEVELS-2];
                    for (unsigned int i=0; i<avPVSsec[0][LEVELS-2].size(); i++)
                        vNextToSearchsec[0].push_back(avPVSsec[0][LEVELS-2][i]);
                    avPVSsec[0][LEVELS-2].clear();
                }
                else
                {
                    for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
                        vNextToSearchsec[0].push_back(avPVSsec[0][LEVELS-2][i]);
                    avPVSsec[0][LEVELS-2].erase(avPVSsec[0][LEVELS-2].begin(), avPVSsec[0][LEVELS-2].begin() + nMoreCoarseNeeded);
                }
            }
            // Now go and attempt to find these points in the second image!
            nFound += SearchForPoints(vNextToSearchsec[0], nCoarseRange, *gvnCoarseSubPixIts, 1);
            // Copy over into the to-be-optimised list.
            for (unsigned int i = 0; i < vNextToSearchsec[0].size(); i ++)
                if (vNextToSearchsec[0][i]->TData.bFound)
                    vIterationSet.push_back(vNextToSearchsec[0][i]);

            if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
            {/// only using the addcam number 0
                cout << "coarse tracking using dual img" << endl;
                mbDidCoarse = true;
                for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
                {
                    if(iter != 0)
                    { // Re-project the points on all but the first iteration.
                        // This should be bases on different camera model.
                        for(unsigned int i=0; i<vIterationSet.size(); i++)
                            if(vIterationSet[i]->TData.bFound)
                            {
                                if (use_seccam_track){//if only try using second img
                                    if (vIterationSet[i]->nFoundCamera)// second img
                                        vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, posesecCamFromWorld[vIterationSet[i]->nFoundCamera -1], mCameraSec[vIterationSet[i]->nFoundCamera - 1].get());
//                                    else
//                                        vIterationSet[i]->TData.bFound = false;
                                }else
                                {
                                    if (vIterationSet[i]->nFoundCamera)// second img
                                        vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, posesecCamFromWorld[vIterationSet[i]->nFoundCamera -1], mCameraSec[vIterationSet[i]->nFoundCamera -1].get());
                                    else
                                        vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
                                }
                            }
                    }
                    for(unsigned int i=0; i<vIterationSet.size(); i++)
                        if(vIterationSet[i]->TData.bFound){
                            if (use_seccam_track){
                                if (vIterationSet[i]->nFoundCamera){
                                    if (poseupdate_cam2)
                                        vIterationSet[i]->TData.CalcJacobian();
                                    else
                                        vIterationSet[i]->TData.CalcJacobiansec(mse3Cam1FromCam2[0].inverse());
                                }
//                                else
//                                    vIterationSet[i]->TData.bFound = false;
                            }else
                            {
                                if (vIterationSet[i]->nFoundCamera)// sec img
                                    vIterationSet[i]->TData.CalcJacobiansec(mse3Cam1FromCam2[0].inverse());
                                else
                                    vIterationSet[i]->TData.CalcJacobian();
                            }
                        }
                    double dOverrideSigma = 0.0;
                    // Hack: force the MEstimator to be pretty brutal
                    // with outliers beyond the fifth iteration.
                    if(iter > 5)
                        dOverrideSigma = 1.0;

                    // Calculate and apply the pose update...
                    Vector<6> v6Update =
                            CalcPoseUpdate(vIterationSet, dOverrideSigma, false, use_seccam_track);
                    if (use_seccam_track && poseupdate_cam2){
                        posesecCamFromWorld[0] = SE3<>::exp(v6Update) * posesecCamFromWorld[0];
                        mse3CamFromWorld = mse3Cam1FromCam2[0] * posesecCamFromWorld[0];
                        for (int cn = 0; cn < AddCamNumber-1; cn ++)
                            posesecCamFromWorld[cn + 1] = mse3Cam1FromCam2[cn + 1].inverse() * mse3CamFromWorld;
                        cout << "using only second cam for coarse tracking.." << endl;
                    }else
                    {
                        mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
                        for (int cn = 0; cn < AddCamNumber; cn ++)
                            posesecCamFromWorld[cn] = mse3Cam1FromCam2[cn].inverse() * mse3CamFromWorld;
//                        SE3<> v6Updatesec = mse3Cam1FromCam2.inverse() * SE3<>::exp(v6Update);
//                        v6Updatesec = v6Updatesec * mse3Cam1FromCam2;
//                        posesecCamFromWorld = v6Updatesec * posesecCamFromWorld;
                    }
                };
            }
        }
        else if (nPvsEnoughFirstImg){// do pose update using only first img pvs
            mbDidCoarse = true;
            for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
            {
                if(iter != 0)
                { // Re-project the points on all but the first iteration.
                    // This should be bases on different camera model.
                    for(unsigned int i=0; i<vIterationSet.size(); i++)
                        if(vIterationSet[i]->TData.bFound){
                            if (vIterationSet[i]->nFoundCamera)// second img
                                vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, posesecCamFromWorld[vIterationSet[i]->nFoundCamera - 1], mCameraSec[vIterationSet[i]->nFoundCamera - 1].get());
                            else
                                vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
                        }
                }
                for(unsigned int i=0; i<vIterationSet.size(); i++)
                    if(vIterationSet[i]->TData.bFound){
                        if (vIterationSet[i]->nFoundCamera)// sec img
                            vIterationSet[i]->TData.CalcJacobiansec(mse3Cam1FromCam2[0].inverse());
                        else
                            vIterationSet[i]->TData.CalcJacobian();
                    }
                double dOverrideSigma = 0.0;
                // Hack: force the MEstimator to be pretty brutal
                // with outliers beyond the fifth iteration.
                if(iter > 5)
                    dOverrideSigma = 1.0;

                // Calculate and apply the pose update...
                Vector<6> v6Update =
                        CalcPoseUpdate(vIterationSet, dOverrideSigma);
                mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
                for (int cn = 0; cn < AddCamNumber; cn ++)
                    posesecCamFromWorld[cn] = mse3Cam1FromCam2[cn].inverse() * mse3CamFromWorld;
            };
        }
        else {
            // not enough high-level PV map points
            //            std::cout << __PRETTY_FUNCTION__ << " not enough high-level PV map points for coarse search" << std::endl;
        }
    }

    // So, at this stage, we may or may not have done a coarse tracking stage.
    // Now do the fine tracking stage. This needs many more points!
    // *similar to the coarse stage, use all pvs on dual img.
    // *still try to limit used points below 1000, master camera img comes first

//    if (bTryCoarse)
//        cout << "Cam pose after coarse: " << mse3CamFromWorld << endl;

    int nFineRange = 10;  // Pixel search range for the fine stage.
    if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
        nFineRange = 5;

    // What patches shall we use this time? The high-level ones are quite important,
    // so do all of these, with sub-pixel refinement.
    int l = LEVELS - 1;
    if (mbDidCoarse){
        for(unsigned int i=0; i<avPVS[l].size(); i++)
            if (avPVS[l][i]->nFoundCamera)// second img
                avPVS[l][i]->TData.ProjectAndDerivs(avPVS[l][i]->v3WorldPos, posesecCamFromWorld[avPVS[l][i]->nFoundCamera - 1], mCameraSec[avPVS[l][i]->nFoundCamera - 1].get());
            else
                avPVS[l][i]->TData.ProjectAndDerivs(avPVS[l][i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
    }
    SearchForPoints(avPVS[l], nFineRange, 8);
    for(unsigned int i=0; i<avPVS[l].size(); i++)
        vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
    // and the second img
    if (mUsingDualImg)
        for (int cn = 0; cn < AddCamNumber; cn ++){
            if (mbDidCoarse)
                for(unsigned int i=0; i<avPVSsec[cn][l].size(); i++){
                    if (avPVSsec[cn][l][i]->nFoundCamera)// second img
                        avPVSsec[cn][l][i]->TData.ProjectAndDerivs(avPVSsec[cn][l][i]->v3WorldPos, posesecCamFromWorld[avPVSsec[cn][l][i]->nFoundCamera - 1], mCameraSec[avPVSsec[cn][l][i]->nFoundCamera - 1].get());
                    else
                        avPVSsec[cn][l][i]->TData.ProjectAndDerivs(avPVSsec[cn][l][i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
                }
            SearchForPoints(avPVSsec[cn][l], nFineRange, 8, 1);
            for(unsigned int i=0; i<avPVSsec[cn][l].size(); i++)
                vIterationSet.push_back(avPVSsec[cn][l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
        }
    // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
    vNextToSearch.clear();
    for(int l=LEVELS - 2; l>=0; l--)
        for(unsigned int i=0; i<avPVS[l].size(); i++)
            vNextToSearch.push_back(avPVS[l][i]);
    // and the second img
    if (mUsingDualImg)
        for (int cn = 0; cn < AddCamNumber; cn ++){
            vNextToSearchsec[cn].clear();
            for(int l=LEVELS - 2; l>=0; l--)
                for(unsigned int i=0; i<avPVSsec[cn][l].size(); i++)
                    vNextToSearchsec[cn].push_back(avPVSsec[cn][l][i]);
        }

    // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
    // ourselves to 1000, and choose these randomly.
    // * randomly chose points from both img
    int numFirstcam = 0;
    int numSeccam = 0;/// a vector when more than one addcam
    for (int i = 0; i < vIterationSet.size(); i ++){
        if (vIterationSet[i]->nFoundCamera)
            numSeccam ++;
        else
            numFirstcam ++;
    }
    static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
    int nFinePatchesToUse = *gvnMaxPatchesPerFrame - numFirstcam;//vIterationSet.size();
    if(nFinePatchesToUse < 0)
        nFinePatchesToUse = 0;
    if((int) vNextToSearch.size() > nFinePatchesToUse)
    {
        random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
        vNextToSearch.resize((int) nFinePatchesToUse); // Chop!
    };
    // and the second img, each chose nFinePatchesToUse/2
    nFinePatchesToUse = *gvnMaxPatchesPerFrame - numSeccam;
    for (int cn = 0; cn < AddCamNumber; cn ++)
        if(mUsingDualImg && (int) vNextToSearchsec[cn].size() > nFinePatchesToUse)// nFinePatchesToUse/2
        {
            random_shuffle(vNextToSearchsec[cn].begin(), vNextToSearchsec[cn].end());
            // TODO: sort the set accordinig to its distance to the camera
            vNextToSearchsec[cn].resize((int) nFinePatchesToUse); // Chop! nFinePatchesToUse/2
        };

    // If we did a coarse tracking stage: re-project and find derivs of fine points
    if(mbDidCoarse)
    {
        for(unsigned int i=0; i<vNextToSearch.size(); i++){
            if (vNextToSearch[i]->nFoundCamera)// second img
                vNextToSearch[i]->TData.ProjectAndDerivs(vNextToSearch[i]->v3WorldPos, posesecCamFromWorld[vNextToSearch[i]->nFoundCamera - 1], mCameraSec[vNextToSearch[i]->nFoundCamera - 1].get());
            else
                vNextToSearch[i]->TData.ProjectAndDerivs(vNextToSearch[i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
        }
        if (mUsingDualImg){
            for (int cn = 0; cn < AddCamNumber; cn ++)
                for(unsigned int i=0; i<vNextToSearchsec[cn].size(); i++){
                    if (vNextToSearchsec[cn][i]->nFoundCamera)// second img
                        vNextToSearchsec[cn][i]->TData.ProjectAndDerivs(vNextToSearchsec[cn][i]->v3WorldPos, posesecCamFromWorld[vNextToSearchsec[cn][i]->nFoundCamera - 1], mCameraSec[vNextToSearchsec[cn][i]->nFoundCamera - 1].get());
                    else
                        vNextToSearchsec[cn][i]->TData.ProjectAndDerivs(vNextToSearchsec[cn][i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
                }
        }
    }

    // Find fine points in image:
    SearchForPoints(vNextToSearch, nFineRange, 0);
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
        if (vNextToSearch[i]->TData.bFound)//yang
            vIterationSet.push_back(vNextToSearch[i]);

    // and in the second image:
    for (int cn = 0; cn < AddCamNumber; cn ++)
        if (mUsingDualImg && vNextToSearchsec[cn].size())
        {
            SearchForPoints(vNextToSearchsec[cn], nFineRange, 0, 1);
            // And attach them all to the end of the optimisation-set.
            for(unsigned int i=0; i<vNextToSearchsec[cn].size(); i++)
                if (vNextToSearchsec[cn][i]->TData.bFound)
                    vIterationSet.push_back(vNextToSearchsec[cn][i]);
            use_seccam_track = use_seccamonly;
        }
    // TODO: if we use timestamp to interpolate image measurements from multiple un-sync'ed kinects,
    // vIterationSet of the last frame needs to be stored.

    // Again, ten gauss-newton pose update iterations.
    // TODO: v6LastUpdate for second img
    Vector<6> v6LastUpdate;
    Vector<6> v6LastUpdatesec[AddCamNumber];
    v6LastUpdate = Zeros;
    Vector<6> v6cam2camerror = Zeros;// initialised as zero
    SE3<> mse3Cam1FromCam2Update[AddCamNumber];
    for (int cn = 0; cn < AddCamNumber; cn ++){
        v6LastUpdatesec[cn] = Zeros;
        mse3Cam1FromCam2Update[cn] = mse3Cam1FromCam2[cn];
    }
    double shiftcut = 1.0;
    for(int iter = 0; iter<10; iter++)
    {
        bool bNonLinearIteration = true; // For a bit of time-saving: don't do full nonlinear
        // reprojection at every iteration - it really isn't necessary!
//        if(iter == 0 || iter == 4 || iter == 9)
//            bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
//        else                            // iterations is for M-Estimator convergence rather than
//            bNonLinearIteration = false;  // linearisation effects.
        //  TODO: decide erriter based on the number of valid points from the second cameras
        bool erriter = TrackCam2camError && (iter>=1);
        if(iter != 0)   // Either way: first iteration doesn't need projection update.
        {
            if(bNonLinearIteration)
            {
                for(unsigned int i=0; i<vIterationSet.size(); i++)
                    if(vIterationSet[i]->TData.bFound)
                    {
                        if (use_seccam_track){//if only try using second img
                            if (vIterationSet[i]->nFoundCamera)// second img
                                vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, posesecCamFromWorld[vIterationSet[i]->nFoundCamera - 1], mCameraSec[vIterationSet[i]->nFoundCamera - 1].get());
//                            else
//                                vIterationSet[i]->TData.bFound = false;
                        }else
                        {
                            if (vIterationSet[i]->nFoundCamera)// second img
                                vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, posesecCamFromWorld[vIterationSet[i]->nFoundCamera - 1], mCameraSec[vIterationSet[i]->nFoundCamera - 1].get());
                            else
                                vIterationSet[i]->TData.ProjectAndDerivs(vIterationSet[i]->v3WorldPos, mse3CamFromWorld, mCamera.get());
                        }
                    }
            }
            else if (false) /// not using LinearUpdate in multicam cases
            {
                for(unsigned int i=0; i<vIterationSet.size(); i++)
                    if(vIterationSet[i]->TData.bFound)
                    {
                        if (vIterationSet[i]->nFoundCamera)// second img
                            vIterationSet[i]->TData.LinearUpdate(v6LastUpdatesec[vIterationSet[i]->nFoundCamera - 1]);
                        else
                            vIterationSet[i]->TData.LinearUpdate(v6LastUpdate);
                    }
            };
        }

        if(bNonLinearIteration)
            for(unsigned int i=0; i<vIterationSet.size(); i++)
                if(vIterationSet[i]->TData.bFound){
                    if (use_seccam_track){
                        if (vIterationSet[i]->nFoundCamera)
                        {
                            if (poseupdate_cam2)
                                vIterationSet[i]->TData.CalcJacobian();
                            else
                                vIterationSet[i]->TData.CalcJacobiansec(mse3Cam1FromCam2Update[vIterationSet[i]->nFoundCamera - 1].inverse());
                        }
//                        else
//                            vIterationSet[i]->TData.bFound = false;
                    }else
                    {
                        if (vIterationSet[i]->nFoundCamera)// sec img
                        {
                            // if we want to track the error of cam2cam,
                            // we should not do this without an initial estimation of cam1 poseupdate
                            // so this should be done from the second iteration.
                            // it's fine for poseupdate calculation, since we derivate the
                            // error at cam2cam error being zeros.
                            if (!erriter)
                                vIterationSet[i]->TData.CalcJacobiansec(mse3Cam1FromCam2Update[vIterationSet[i]->nFoundCamera-1].inverse());
                            else
                                vIterationSet[i]->TData.CalcJacobianWithErrorSec(
                                            mse3Cam1FromCam2Update[vIterationSet[i]->nFoundCamera].inverse());//,
//                                            SE3<>::exp(v6LastUpdate),
//                                            SE3<>::exp(v6cam2camerror));
                        }
                        else
                        {
                            if (!erriter)
                                vIterationSet[i]->TData.CalcJacobian();
                            else
                                vIterationSet[i]->TData.CalcJacobianWithError();
                        }
                    }
                }

        // Again, an M-Estimator hack beyond the fifth iteration.
        double dOverrideSigma = 0.0;
        if(iter > 5)
            dOverrideSigma = 16.0;

        // Calculate and update pose; also store update vector for linear iteration updates.
        if (false){ /// Never track cam2cam error, otherwise update the code related to multi-cam
            Vector<12> v12Update =
                    CalcPoseUpdateDualCam(vIterationSet, dOverrideSigma, iter==9, use_seccam_track);
            Vector<6> v6Update = v12Update.slice(0, 6);
            v6cam2camerror = v12Update.slice(6, 6);
            if (use_seccam_track && poseupdate_cam2){
                cerr<<"Cannot track cam2cam error while only using second camera!" << endl;
            }else
            {
                mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld; // shiftcut*
                mse3Cam1FromCam2Update[0] = SE3<>::exp(v6cam2camerror) * mse3Cam1FromCam2Update[0].inverse(); // shiftcut*
                mse3Cam1FromCam2Update[0] = mse3Cam1FromCam2Update[0].inverse();
                posesecCamFromWorld[0] = mse3Cam1FromCam2Update[0].inverse() * mse3CamFromWorld;

                v6LastUpdate = v6Update;
                v6LastUpdatesec[0] = SE3<>::ln((mse3Cam1FromCam2Update[0].inverse() * SE3<>::exp(v6Update)) * (mse3Cam1FromCam2Update[0]*SE3<>::exp(v6cam2camerror)));// need to change to v6 first!
            }
        }else
        {
            Vector<6> v6Update =
                    CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9, use_seccam_track);
            if (use_seccam_track && poseupdate_cam2){
                posesecCamFromWorld[0] = SE3<>::exp(v6Update) * posesecCamFromWorld[0];
                mse3CamFromWorld = mse3Cam1FromCam2[0] * posesecCamFromWorld[0];
                cout << "using only second cam for tracking.." << endl;
                v6LastUpdatesec[0] = v6Update;
                v6LastUpdate = SE3<>::ln(mse3Cam1FromCam2[0] * SE3<>::exp(v6Update) * mse3Cam1FromCam2[0].inverse());// need to change to v6 first!
            }else
            {
                mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
                v6LastUpdate = v6Update;
                for (int cn = 0; cn < AddCamNumber; cn ++) {
                    posesecCamFromWorld[cn] = mse3Cam1FromCam2[cn].inverse() * mse3CamFromWorld;
                    v6LastUpdatesec[cn] = SE3<>::ln((mse3Cam1FromCam2[cn].inverse() * SE3<>::exp(v6Update)) * mse3Cam1FromCam2[cn]);// need to change to v6 first!
                }
                //            SE3<> v6Updatesec = mse3Cam1FromCam2.inverse() * SE3<>::exp(v6Update);
                //            v6Updatesec = v6Updatesec * mse3Cam1FromCam2;
                //            posesecCamFromWorld = v6Updatesec * posesecCamFromWorld;

            }
        }
    };

    // Update the current keyframe with info on what was found in the frame.
    // Strictly speaking this is unnecessary to do every frame, it'll only be
    // needed if the KF gets added to MapMaker. Do it anyway.
    // Export pose to current keyframe:
    mCurrentKF->se3CfromW = mse3CamFromWorld;
//    if (mUsingDualImg)
    for (int cn = 0; cn < AddCamNumber; cn ++)
    {
        mCurrentKFsec[cn]->se3CfromW = posesecCamFromWorld[cn];
        mCurrentKF->se3Cam2fromCam1 = SE3<>();
        mCurrentKFsec[cn]->se3Cam2fromCam1 = mse3Cam1FromCam2Update[cn].inverse();
        mse3CamFromWorldsec[cn] = posesecCamFromWorld[cn];
    }
//    mse3Cam1FromCam2 = mse3Cam1FromCam2Update;
//    trackerlog << "Tracked pose: \n" << mse3CamFromWorld << endl;

    // Record successful measurements. Use the KeyFrame-Measurement struct for this.
    mCurrentKF->mMeasurements.clear();
    if (mUsingDualImg)
        for (int cn = 0; cn < AddCamNumber; cn ++)
        mCurrentKFsec[cn]->mMeasurements.clear();
    for(vector<boost::shared_ptr<MapPoint> >::iterator it = vIterationSet.begin();
        it!= vIterationSet.end();
        it++)
    {
        if(! (*it)->TData.bFound)
            continue;
        Measurement m;
        m.v2RootPos = (*it)->TData.v2Found;
        m.nLevel = (*it)->TData.nSearchLevel;
        m.bSubPix = (*it)->TData.bDidSubPix;
        m.dDepth = (*it)->TData.dFoundDepth;
        m.Source =  Measurement::SRC_TRACKER;
        if ((*it)->nFoundCamera)// second img
            mCurrentKFsec[(*it)->nFoundCamera-1]->mMeasurements[*it] = m;
        else
            mCurrentKF->mMeasurements[*it] = m;
    }

    // Finally, find the mean scene depth from tracked features
    // also should based on dual image individually
    {
        double dSum = 0;
        double dSumSq = 0;
        int nNum = 0;
        double dSumsec[AddCamNumber] ;
        double dSumSqsec[AddCamNumber] ;
        int nNumsec[AddCamNumber];
        for (int cn = 0; cn < AddCamNumber; cn ++){
            dSumsec[cn] = 0;
            dSumSqsec[cn] = 0;
            nNumsec[cn] = 0;
        }

        std::vector<double> buffvec;//eth
        std::vector<double> buffvecsec[AddCamNumber];//eth
        for(vector<boost::shared_ptr<MapPoint> >::iterator it = vIterationSet.begin();
            it!= vIterationSet.end();
            it++)
            if((*it)->TData.bFound)
            {
                double z = (*it)->TData.v3Cam[2];
                if (!(*it)->nFoundCamera){
                    dSum+= z;
                    dSumSq+= z*z;

                    buffvec.push_back(z);//eth

                    nNum++;
                }
                else{
                    dSumsec[(*it)->nFoundCamera-1]+= z;
                    dSumSqsec[(*it)->nFoundCamera-1]+= z*z;

                    buffvecsec[(*it)->nFoundCamera-1].push_back(z);//eth

                    nNumsec[(*it)->nFoundCamera-1]++;
                }
            };
//        cout << "cam0 and cam1 nNum: " << nNum << ", " << nNumsec << endl;
        if(nNum > 20)
        {
            mCurrentKF->dSceneDepthMean = dSum/nNum;
            mCurrentKF->dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF->dSceneDepthMean) * (mCurrentKF->dSceneDepthMean));
//            cout << "cam0 depth updated: " << dSum/nNum << endl;
            //eth{
            std::vector<double>::iterator first = buffvec.begin();
            std::vector<double>::iterator last = buffvec.end();
            std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
            std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
            mCurrentKF->dSceneDepthMedian = *middle;
            //}
        }
        for (int cn = 0; cn < AddCamNumber; cn ++)
        if(nNumsec[cn] > 20)
        {
            mCurrentKFsec[cn]->dSceneDepthMean = dSumsec[cn]/nNumsec[cn];
            mCurrentKFsec[cn]->dSceneDepthSigma = sqrt((dSumSqsec[cn] / nNumsec[cn]) - (mCurrentKFsec[cn]->dSceneDepthMean) * (mCurrentKFsec[cn]->dSceneDepthMean));
            //eth{
            std::vector<double>::iterator first = buffvecsec[cn].begin();
            std::vector<double>::iterator last = buffvecsec[cn].end();
            std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
            std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
            mCurrentKFsec[cn]->dSceneDepthMedian = *middle;
            //}
        }
//        else if(nNumsec > 2)// scene depth changes a lot in some cases
//        {
//            mCurrentKFsec->dSceneDepthMean = dSumsec/nNumsec *2;
//            mCurrentKFsec->dSceneDepthSigma = sqrt((dSumSqsec / nNumsec) - (mCurrentKFsec->dSceneDepthMean) * (mCurrentKFsec->dSceneDepthMean))*2;
//        }
    }
}

// Calculate covariance matrix of last pose
Matrix<6, 6> Tracker::GetPoseCovariance() const {
    Cholesky<6> decomp0(wls.get_C_inv());
    //        Matrix<6, 6> decomp = TooN::SVD<6>(wls.get_C_inv()).get_pinv();
    //        cout<< "decomping wls: " << decomp.get_inverse()[0][0] << endl;
    //        for (int i = 0; i < 6; i++){
    //            for (int j = 0; j < 6; j ++)
    //                cout << decomp0.get_inverse()[i][j] << ", ";
    //        }
    //        cout << endl;
    //        for (int i = 0; i < 6; i++){
    //            for (int j = 0; j < 6; j ++)
    //                cout << decomp[i][j] << ", ";
    //        }
    //        cout << endl;

    if (wls.get_C_inv()(0, 0)*wls.get_C_inv()(1, 0)*wls.get_C_inv()(2, 0)
            *wls.get_C_inv()(3, 0)*wls.get_C_inv()(4, 0)*wls.get_C_inv()(5, 0) == 0.0)
        return Zeros;
    else
        return decomp0.get_inverse();
    //            return decomp;
}

// Find points in the image. Uses the PatchFinder struct stored in TrackerData
// yang, should be able to search in individual dual image.
// we should not attempt to search for map points, produced by one-camera images,
// in another one-camera images
int Tracker::SearchForPoints(vector<boost::shared_ptr<MapPoint> >& vTD, int nRange, int nSubPixIts, int nCamera)
{
    static gvar3<int> gvnUseDepthTracking("Tracker.UseDepth",0,SILENT);
    static gvar3<int> gvnUse3DTracking("Tracker.Use3D",0,SILENT);
    int nFound = 0;

    for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
    {
        // First, attempt a search at pixel locations which are FAST corners.
        // (PatchFinder::FindPatchCoarse)
        TrackerData &TD = vTD[i]->TData;
        PatchFinder &Finder = TD.Finder;
        Finder.MakeTemplateCoarseCont(*vTD[i]);
        if(Finder.TemplateBad())
        {
            TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
            continue;
        }
        manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessment

        bool bFound = false;
        if (!vTD[i]->nFoundCamera)
            bFound = Finder.FindPatchCoarse(ir(TD.v2Image), *mCurrentKF, nRange);
        else// search on the second image
            bFound = Finder.FindPatchCoarse(ir(TD.v2Image), *mCurrentKFsec[vTD[i]->nFoundCamera - 1], nRange);
        TD.bSearched = true;
        if(!bFound)
        {
            TD.bFound = false;
            continue;
        }

        TD.dFoundDepth = Finder.GetCoarseDepth();

        TD.bFound = true;
        if (!vTD[i]->nFoundCamera)
            TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
        else
            TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());//considering cam2cam calibration error

        nFound++;
        manMeasFound[Finder.GetLevel()]++;

        // Found the patch in coarse search - are Sub-pixel iterations wanted too?
        if(nSubPixIts > 0)
        {
            TD.bDidSubPix = true;
            Finder.MakeSubPixTemplate();
            bool bSubPixConverges= false;
            if (!vTD[i]->nFoundCamera)
                bSubPixConverges = Finder.IterateSubPixToConvergence(*mCurrentKF, nSubPixIts);
            else
                bSubPixConverges = Finder.IterateSubPixToConvergence(*mCurrentKFsec[vTD[i]->nFoundCamera - 1], nSubPixIts);
            if(!bSubPixConverges)
            { // If subpix doesn't converge, the patch location is probably very dubious!
                TD.bFound = false;
                nFound--;
                manMeasFound[Finder.GetLevel()]--;
                continue;
            }
            TD.v2Found = Finder.GetSubPixPos();
        }
        else
        {
            TD.v2Found = Finder.GetCoarsePosAsVector();
            TD.bDidSubPix = false;
        }

        if ((TD.dFoundDepth > 0.0)&& TD.dFoundDepth < mMaxDepth) {
            if (!vTD[i]->nFoundCamera)
                TD.v3Found = TD.dFoundDepth*unproject(mCamera->UnProject(ir(TD.v2Found)));
            else
                TD.v3Found = TD.dFoundDepth*unproject(mCameraSec[vTD[i]->nFoundCamera - 1]->UnProject(ir(TD.v2Found)));

            if (*gvnUseDepthTracking == 1 || *gvnUse3DTracking == 1) {
                static gvar3<double> gvdDepthErrorScale("Tracker.DepthErrorScale",0.0025,SILENT);
                TD.dSqrtInvDepthNoise = 1.0/(TD.dFoundDepth*TD.dFoundDepth*(*gvdDepthErrorScale));
            }

        }
    }
    return nFound;
};

//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
Vector<6> Tracker::CalcPoseUpdate(vector<boost::shared_ptr<MapPoint> > vTD, double dOverrideSigma, bool bMarkOutliers, bool debug)
{
    static  gvar3<int> gvnUseDepthTracking("Tracker.UseDepth",0,SILENT);
    static gvar3<int> gvnUse3DTracking("Tracker.Use3D",0,SILENT);
    static gvar3<int> gvnUseDiffMEstimate("Tracker.UseDiffMEstimate",0,SILENT);
    static bool bUseDiffMEstimate = *gvnUseDiffMEstimate && *gvnUseDepthTracking;


    // Which M-estimator are we using?
    int nEstimator = 0;
    static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
    if(*gvsEstimator == "Tukey")
        nEstimator = 0;
    else if(*gvsEstimator == "Cauchy")
        nEstimator = 1;
    else if(*gvsEstimator == "Huber")
        nEstimator = 2;
    else if(*gvsEstimator == "LeastSquares")
        nEstimator = 3;
    else
    {
        cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber, LeastSquares" << endl;
        nEstimator = 0;
        *gvsEstimator = "Tukey";
    };

    // Find the covariance-scaled reprojection error for each measurement.
    // Also, store the square of these quantities for M-Estimator sigma squared estimation.
    vector<double> vdErrorSquared;
    vector<double> vdErrorSquaredDepth;
    for(unsigned int f=0; f<vTD.size(); f++)
    {
        TrackerData &TD = vTD[f]->TData;
        if(!TD.bFound)
            continue;
        if (!vTD[f]->nSourceCamera && debug)
            continue;

        TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
        if (*gvnUseDepthTracking == 1 && TD.dFoundDepth > 0.0) {
            // 2d error:
            vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
            // depth error:
            double depthError_CovScaled = TD.dSqrtInvDepthNoise*(TD.dFoundDepth - TD.v3Cam[2]);
            if (!bUseDiffMEstimate)
                vdErrorSquared.push_back(depthError_CovScaled*depthError_CovScaled);
            else
                vdErrorSquaredDepth.push_back(depthError_CovScaled*depthError_CovScaled);
        } else if (*gvnUse3DTracking == 1 && TD.dFoundDepth > 0.0) {
            TD.v3Error_CovScaled = (TD.dSqrtInvDepthNoise+TD.dSqrtInvNoise)*(TD.v3Found - TD.v3Cam);
            vdErrorSquared.push_back(TD.v3Error_CovScaled * TD.v3Error_CovScaled);
        } else {
            vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
        }
    };

    // No valid measurements? Return null update.
    if(vdErrorSquared.size() == 0)
        return makeVector( 0,0,0,0,0,0);

    // What is the distribution of errors?
    double dSigmaSquared;
    double dSigmaSquaredDepth;
    if(dOverrideSigma > 0)
        dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
    else
    {
        if (nEstimator == 0){
            dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
            if (bUseDiffMEstimate && vdErrorSquaredDepth.size()>0)
                dSigmaSquaredDepth = Tukey::FindSigmaSquared(vdErrorSquaredDepth);
        }
        else if(nEstimator == 1)
            dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
        else if(nEstimator == 2)
            dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
        else if (nEstimator == 3)
            dSigmaSquared = LeastSquares::FindSigmaSquared(vdErrorSquared);
        else
            assert(false);
    }

    // The TooN WLSCholesky class handles reweighted least squares.
    // It just needs errors and jacobians.

    wls.clear();
    wls.add_prior(100.0); // Stabilising prior
    for(unsigned int f=0; f<vTD.size(); f++)
    {
        TrackerData &TD = vTD[f]->TData;
        if(!TD.bFound)
            continue;
        if (!vTD[f]->nSourceCamera && debug)
            continue;

        double dErrorSq;
        double dWeight;
        double dErrorSqDepth;
        double dWeightDepth;

        bool bUse3DTracking = (*gvnUse3DTracking == 1) && (TD.dFoundDepth > 0.0);
        bool bUseDiffErr = bUseDiffMEstimate && (TD.dFoundDepth > 0.0);

        if (bUse3DTracking) {
            Vector<3> &v3 = TD.v3Error_CovScaled;
            dErrorSq = v3 * v3;
            //std::cout << "using 3d" << std::endl;
        } else {
            Vector<2> &v2 = TD.v2Error_CovScaled;
            dErrorSq = v2 * v2;
        }

        if(nEstimator == 0)
            dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
        else if(nEstimator == 1)
            dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
        else if (nEstimator == 2)
            dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
        else if (nEstimator == 3)
            dWeight= LeastSquares::Weight(dErrorSq, dSigmaSquared);
        else
            assert(false);

//        if (vTD[f]->nSourceCamera) /// TODO: remove this line!
//            dWeight = dWeight *0.6;

        // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
        if(dWeight == 0.0)
        {
            if(bMarkOutliers)
                vTD[f]->nMEstimatorOutlierCount++;
            continue;
        }
        else
            if(bMarkOutliers)
                vTD[f]->nMEstimatorInlierCount++;
        if (bUse3DTracking) {
            Matrix<3,6> &m36Jac = TD.m36Jacobian;
            Vector<3> &v3 = TD.v3Error_CovScaled;
            wls.add_mJ(v3[0], TD.dSqrtInvNoise * m36Jac[0], dWeight);
            wls.add_mJ(v3[1], TD.dSqrtInvNoise * m36Jac[1], dWeight);
            wls.add_mJ(v3[2], TD.dSqrtInvNoise * m36Jac[2], dWeight);
        } else {
            Matrix<2,6> &m26Jac = TD.m26Jacobian;
            Vector<2> &v2 = TD.v2Error_CovScaled;
            wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
            wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
            // TODO: also add depth measurement in the tracker
        }

    }
    wls.compute();

    return wls.get_mu();
}

//output vector: pose update + cam2cam calibration error update
Vector<12> Tracker::CalcPoseUpdateDualCam(vector<boost::shared_ptr<MapPoint> > vTD, double dOverrideSigma, bool bMarkOutliers, bool debug)
{
    static  gvar3<int> gvnUseDepthTracking("Tracker.UseDepth",0,SILENT);
    static gvar3<int> gvnUse3DTracking("Tracker.Use3D",0,SILENT);

    // Which M-estimator are we using?
    int nEstimator = 0;
    static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
    if(*gvsEstimator == "Tukey")
        nEstimator = 0;
    else if(*gvsEstimator == "Cauchy")
        nEstimator = 1;
    else if(*gvsEstimator == "Huber")
        nEstimator = 2;
    else if(*gvsEstimator == "LeastSquares")
        nEstimator = 3;
    else
    {
        cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber, LeastSquares" << endl;
        nEstimator = 0;
        *gvsEstimator = "Tukey";
    };

    // Find the covariance-scaled reprojection error for each measurement.
    // Also, store the square of these quantities for M-Estimator sigma squared estimation.
    vector<double> vdErrorSquared;
    for(unsigned int f=0; f<vTD.size(); f++)
    {
        TrackerData &TD = vTD[f]->TData;
        if(!TD.bFound)
            continue;
        if (!vTD[f]->nSourceCamera && debug)
            continue;

        TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
        if (*gvnUseDepthTracking == 1 && TD.dFoundDepth > 0.0) {
            // 2d error:
            vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
            // depth error:
            double depthError_CovScaled = TD.dSqrtInvDepthNoise*(TD.dFoundDepth - TD.v3Cam[2]);
            vdErrorSquared.push_back(depthError_CovScaled*depthError_CovScaled);
        } else if (*gvnUse3DTracking == 1 && TD.dFoundDepth > 0.0) {
            TD.v3Error_CovScaled = (TD.dSqrtInvDepthNoise+TD.dSqrtInvNoise)*(TD.v3Found - TD.v3Cam);
            vdErrorSquared.push_back(TD.v3Error_CovScaled * TD.v3Error_CovScaled);
        } else {
            vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
        }
    };

    // No valid measurements? Return null update.
    if(vdErrorSquared.size() == 0)
        return makeVector( 0,0,0,0,0,0, 0,0,0,0,0,0);

    // What is the distribution of errors?
    double dSigmaSquared;
    if(dOverrideSigma > 0)
        dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
    else
    {
        if (nEstimator == 0)
            dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
        else if(nEstimator == 1)
            dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
        else if(nEstimator == 2)
            dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
        else if (nEstimator == 3)
            dSigmaSquared = LeastSquares::FindSigmaSquared(vdErrorSquared);
        else
            assert(false);
    }

    // The TooN WLSCholesky class handles reweighted least squares.
    // It just needs errors and jacobians.

    wls2.clear();
    wls2.add_prior(100.0); // Stabilising prior
    for(unsigned int f=0; f<vTD.size(); f++)
    {
        TrackerData &TD = vTD[f]->TData;
        if(!TD.bFound)
            continue;
        if (!vTD[f]->nSourceCamera && debug)
            continue;

        double dErrorSq;
        double dWeight;

        bool bUse3DTracking = (*gvnUse3DTracking == 1) && (TD.dFoundDepth > 0.0);

        if (bUse3DTracking) {
            Vector<3> &v3 = TD.v3Error_CovScaled;
            dErrorSq = v3 * v3;
            //std::cout << "using 3d" << std::endl;
        } else {
            Vector<2> &v2 = TD.v2Error_CovScaled;
            dErrorSq = v2 * v2;
        }

        if(nEstimator == 0)
            dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
        else if(nEstimator == 1)
            dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
        else if (nEstimator == 2)
            dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
        else if (nEstimator == 3)
            dWeight= LeastSquares::Weight(dErrorSq, dSigmaSquared);
        else
            assert(false);

        // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
        if(dWeight == 0.0)
        {
            if(bMarkOutliers)
                vTD[f]->nMEstimatorOutlierCount++;
            continue;
        }
        else
            if(bMarkOutliers)
                vTD[f]->nMEstimatorInlierCount++;
//        if (bUse3DTracking) {
//            Matrix<3,6> &m36Jac = TD.m36Jacobian;
//            Vector<3> &v3 = TD.v3Error_CovScaled;
//            wls2.add_mJ(v3[0], TD.dSqrtInvNoise * m36Jac[0], dWeight);
//            wls2.add_mJ(v3[1], TD.dSqrtInvNoise * m36Jac[1], dWeight);
//            wls2.add_mJ(v3[2], TD.dSqrtInvNoise * m36Jac[2], dWeight);
//        } else
        {
            Matrix<2,12> &m212Jac = TD.m212JacobianWE;
            Vector<2> &v2 = TD.v2Error_CovScaled;
            wls2.add_mJ(v2[0], TD.dSqrtInvNoise * m212Jac[0], dWeight); // These two lines are currently
            wls2.add_mJ(v2[1], TD.dSqrtInvNoise * m212Jac[1], dWeight); // the slowest bit of poseits
        }

    }
    wls2.compute();

    return wls2.get_mu();
}//*/

// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel()
{
    mse3StartPos = mse3CamFromWorld;
    Vector<6> v6Velocity = mv6CameraVelocity;
    Vector<6> v6Velocitysec[AddCamNumber];
    for (int i = 0; i < AddCamNumber; i ++)
        v6Velocitysec[i] = mv6CameraVelocitysec[i];

    if(mbUseSBIInit)
    {
        if (mUsingDualImg)
        {
            // extrinsic param of the second camera need to be considered first.
            // use the mean rot estimation of dual images as the final estimation.
            // TODO: should find a better way to decide which estimate to use
            v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
            for (int i = 0; i < AddCamNumber; i ++){
                v6Velocitysec[i].slice<3,3>() = mv6SBIRotSec[i].slice<3,3>();
                v6Velocitysec[i][0] = 0.0;
                v6Velocitysec[i][1] = 0.0;
            }
        }
        else
            v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
        v6Velocity[0] = 0.0;
        v6Velocity[1] = 0.0;
    }
    mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
    mse3CamFromWorldPub = mse3CamFromWorld;
    if (mUsingDualImg)
        for (int i = 0; i < AddCamNumber; i ++)
            mse3CamFromWorldsec[i] = SE3<>::exp(v6Velocitysec[i]) * mse3CamFromWorldsec[i];
    //    else// trust the forward looking camera
//        mse3CamFromWorld = (mse3Cam1FromCam2*(SE3<>::exp(v6Velocitysec))) * mse3StartPos;
};

// Use the predicted pose from the EKF.
// So no motion model need to be considered here?
void Tracker::ApplyMotionModel_EKF()
{
    if (pose_ekf_get){//TODO: only use this when pose prediction reliable
        mse3StartPos = mse3CamFromWorld;
        mse3CamFromWorld = se3CfromW_predicted_ekf; // use the predicted pose of the EKF
    }
    else {
        mse3StartPos = mse3CamFromWorld;
        Vector<6> v6Velocity = mv6CameraVelocity;
        if(mbUseSBIInit)
        {
            v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
            v6Velocity[0] = 0.0;
            v6Velocity[1] = 0.0;
        }
        mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
    }
    std::cout << "Motion model from EKF pose: " << mse3CamFromWorld.get_translation()[0] << ", " << mse3CamFromWorld.get_translation()[1] << ", " << mse3CamFromWorld.get_translation()[2] << ", " <<std::endl;
};

// assume speed unchanged, which could be updated upon a scale later
// assume translation to reference kf unchanged, which should be updated upon a scale too
void Tracker::MotionModelUpdateByGMap()
{
//    cout << "old current cam pose: \n" << mse3CamFromWorld << endl;
//    trackerlog << "old current cam pose: \n" << mse3CamFromWorld << endl;
    // TODO: apply scale update when SIM3 PGO is used
    SE3<> relpos2refkf = mse3CamFromWorld * mMapMaker.mse3LatestKFpose.inverse();
    SE3<> newkfpose;
    bool posefound = false;
    for (int i = mMap.vpKeyFrames.size()-1; i >=0; i --)
        if (mMap.vpKeyFrames[i]->id == mMapMaker.lastKFid){
            newkfpose = mMap.vpKeyFrames[i]->se3CfromW;
            posefound = true;
            break;
        }
    if (posefound){
        SE3<> se3Old = mse3CamFromWorld;
        mse3CamFromWorld = relpos2refkf * newkfpose;
//        trackerlog << "rel pose: \n" << relpos2refkf << endl;
//        trackerlog << "newkf pose: \n" << newkfpose << endl;
//        cout << "Updated current cam pose: \n" << mse3CamFromWorld << endl;
//        trackerlog << "Updated current cam pose: \n" << mse3CamFromWorld << endl;

        // and update the startpose, for velocity update
        relpos2refkf = mse3StartPos * mMapMaker.mse3LatestKFpose.inverse();
        mse3StartPos = relpos2refkf * newkfpose;

        mv6CameraVelocity = Zeros;
        // TODO: apply scale update to speed when SIM3 PGO is used
//        SE3<> se3NewFromOld = mse3CamFromWorld * se3Old.inverse();
//        mv6CameraVelocity *=
        cout << "Motion model updated!!!!!!!!!!" << endl;
    }
    mbJustRecoveredSoUseCoarse = true;
}

// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel()
{
    SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
    Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);
    Vector<6> v6OldVel = mv6CameraVelocity;

    mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);//0.9
    mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);

    // Also make an estimate of this which has been scaled by the mean scene depth.
    // This is used to decide if we should use a coarse tracking stage.
    // We can tolerate more translational vel when far away from scene!
    Vector<6> v6 = mv6CameraVelocity;
    v6.slice<0,3>() *= 1.0 / mCurrentKF->dSceneDepthMean;
    mdMSDScaledVelocityMagnitude = sqrt(v6*v6);

    if (mUsingDualImg)
        for (int i = 0; i < AddCamNumber; i ++){
            v6Motion = SE3<>::ln(mCurrentKFsec[i]->se3Cam2fromCam1*se3NewFromOld*mCurrentKFsec[i]->se3Cam2fromCam1.inverse()); // abusing v6Motion
            mv6CameraVelocitysec[i] = 0.9 * (0.5 * v6Motion + 0.5 * mv6CameraVelocitysec[i]);//0.9
        }
}

void Tracker::MotionCheck()
{
    static gvar3<double> maxPoseDiff("Tracker.MaxPoseDiff", 1.0, SILENT);
    static gvar3<double> maxAngularDiff("Tracker.MaxAngularDiff", 15.0, SILENT);

    double transdiff = sqrt((mse3CamFromWorld.get_translation() - mse3StartPos.get_translation())*(mse3CamFromWorld.get_translation() - mse3StartPos.get_translation()));

    SE3<> se3C2fromC1 = mse3CamFromWorld*mse3StartPos.inverse();
    Matrix<3,3> R = se3C2fromC1.get_rotation().get_matrix();
    double trace = R(0,0) + R(1,1) + R(2,2);
    double eulerdiff = acos((trace-1.0)/2);

    if (transdiff > *maxPoseDiff ||
            eulerdiff > (*maxAngularDiff)*M_PI/180.0 ){
        mse3CamFromWorld = mse3StartPos;
        ApplyMotionModel();
        std::cout << "TRACKING FAILED, USING LAST POSE. " << std::endl;
        return;
    }
    mse3CamFromWorldPub = mse3CamFromWorld;
}

// Time to add a new keyframe? The MapMaker handles most of this.
// *this is a main collection with the mapper thread
// *kfs should be treated seperately, but we simply add them at the same poses
void Tracker::AddNewKeyFrame()
{
    if (mUsingDualImg){
        for (int i = 0; i < AddCamNumber; i ++)
            if (!mCurrentKFsec[i]->bNewsec)
                return;// Now we force the system to add kfs from multi-cam together

        if (mMapMaker.AddKeyFrame(*mCurrentKF)) {
            mnLastKeyFrameDropped = mnFrame;
            mnKeyFrames++;
        } // else: mapmaker chose to ignore this keyframe, e.g. because mapping is disabled
        for (int i = 0; i < AddCamNumber; i ++) {
            if (!mCurrentKFsec[i]->bNewsec)
                continue;

            if (mMapMaker.AddKeyFrameSec(*mCurrentKFsec[i])){
                mnLastKeyFrameDropped = mnFrame;
                mnKeyFramessec[i]++;
            }
        }
    }
    else {
        if (mMapMaker.AddKeyFrame(*mCurrentKF)) {
            mnLastKeyFrameDropped = mnFrame;
            mnKeyFrames++;
        } // else: mapmaker chose to ignore this keyframe, e.g. because mapping is disabled
    }
}

// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality()
{
    int nTotalAttempted = 0;
    int nTotalFound = 0;
    int nLargeAttempted = 0;
    int nLargeFound = 0;

    for(int i=0; i<LEVELS; i++)
    {
        nTotalAttempted += manMeasAttempted[i];
        nTotalFound += manMeasFound[i];
        if(i>=2) nLargeAttempted += manMeasAttempted[i];
        if(i>=2) nLargeFound += manMeasFound[i];
    }

    if(nTotalFound == 0 || nTotalAttempted == 0)
        mTrackingQuality = BAD;
    else
    {
        double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
        double dLargeFracFound;
        if(nLargeAttempted > 10)
            dLargeFracFound = (double) nLargeFound / nLargeAttempted;
        else
            dLargeFracFound = dTotalFracFound;

        static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGoodRatio", 0.3, SILENT);
        static gvar3<int> gviQualityGood("Tracker.TrackingQualityGoodTotal", 200, SILENT);
        static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLostRatio", 0.13, SILENT);

        if(dTotalFracFound > *gvdQualityGood || nTotalFound >= *gviQualityGood)
            mTrackingQuality = GOOD;
        else if(dLargeFracFound < *gvdQualityLost)
            mTrackingQuality = BAD;
        else
            mTrackingQuality = DODGY;
    }

    if(mTrackingQuality == DODGY)
    {
        // Further heuristics to see if it's actually bad, not just dodgy...
        // If the camera pose estimate has run miles away, it's probably bad.
        if(mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
            mTrackingQuality = BAD;
    }

    // constrain the possible attitude when flying a quadrotor.
    // attitude = asin(2*qx*qy + 2*qz*qw)
    // bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)
    //    SE3<> Twc = mse3CamFromWorld.inverse();
    //    if (isflying
    //            && ())
    //        mTrackingQuality == BAD;

    // yang, more strict with DODGY
    if(mTrackingQuality==BAD || (mTrackingQuality==DODGY && mnLostFrames))
        mnLostFrames++;
    else
        mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
    return mMessageForUser.str();
}

void Tracker::CalcSBIRotation()
{
    mpSBILastFrame->MakeJacs();
    pair<SE2<>, double> result_pair;
    result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
    SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera.get());
    mv6SBIRot = se3Adjust.ln();

    if (mUsingDualImg){
        for (int i = 0; i < AddCamNumber; i ++) {
            mpSBILastFramesec[i]->MakeJacs();
            pair<SE2<>, double> result_pair;
            result_pair = mpSBIThisFramesec[i]->IteratePosRelToTarget(*mpSBILastFramesec[i], 6);
            SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCameraSec[i].get());
            mv6SBIRotSec[i] = se3Adjust.ln();
        }
    }
}

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here
