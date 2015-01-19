// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HomographyInit.h"
#include "LevelHelpers.h"
#include "SmallBlurryImage.h"

#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>
#include <cvd/image_convert.h>

#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>

#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <fstream>
#include <algorithm>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ODT/cvdopencv_helper.h>
#include <ODT/homographysolver.h>

#include <cs_geometry/Conversions.h>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace TooN;
using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace ptam;
using namespace backend;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, SLAMSystem &ss, LoopClosing &be, bool bOffline)
    : mMap(m), mSLAM(ss), mbackend_(be), mCamera(CameraModel::CreateCamera()),
      mbOffline(bOffline), mbMappingEnabled(true), newRecentKF(false)
{
    for (int i = 0; i < AddCamNumber; i ++){
        std::auto_ptr<CameraModel> camera_temp (CameraModel::CreateCamera(i + 1));
        mCameraSec[i] = camera_temp;
    }
    mbResetRequested = false;
    Reset();
    if (!bOffline)
        start(); // This CVD::thread func starts the map-maker thread with function run()
    GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
    GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.1, SILENT); // Default to 10cm between keyframes

    mLandingPad = new landing_object;
    isLandingpadPoseGet = false;
    isLandingPoseGetCurrent = false;
    isFinishPadDetection = false;

    needMotionModelUpdate = false;
    debugmarkLoopDetected = false;

    bInputStopped = false;
    bFullBAfinished = false;
    bStopForFullBA = false;

    pos_log_.open("landingpad.log");
    cout <<"logfile in mapping open?: " << pos_log_.is_open() << endl;
    if (pos_log_.is_open()){
        pos_log_.setf(std::ios::fixed, std::ios::floatfield);
        pos_log_.precision(10);
    }
};

void MapMaker::Reset()
{
    // This is only called from within the mapmaker thread...
    mMap.Reset();
    mvFailureQueue.clear();
    while(!mqNewQueue.empty()) mqNewQueue.pop();
    mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
    mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes
    for (int i = 0; i < AddCamNumber; i ++)
    {
        mvpKeyFrameQueueSec[i].clear(); // TODO: actually erase old keyframes
        secKFid[i] = 0;
    }
    mbBundleRunning = false;
    mbBundleConverged_Full = true;
    mbBundleConverged_Recent = true;
    mbResetDone = true;
    mbResetRequested = false;
    mbBundleAbortRequested = false;

    nullObject_keyframe = true;
    nullTracking_frame = true;
    isObject_detected = false;
    nPositive = 0;
    nNegative = 0;
    msubimglt = ImageRef(0, 0);
    msubimgsize = ImageRef(0, 0);
    mPadHomography = cv::Mat::ones(3, 3, CV_64FC1);
    mpadtrackerhomography = TooN::Ones;
    mHlast2thisframe = TooN::Ones;
    mTsubimagelast = TooN::Ones;
    mTsubimagethis = TooN::Ones;

    nAddedKfNoBA = 0;

    imageInputCount = 0;
    lastimagecount = 0;
}

void MapMaker::OfflineStep(bool bDoGlobal)
{
    assert(mbOffline);
    if (mMap.IsGood()) {
        // Any new key-frames to be added?
        if(QueueSize() > 0)
            AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process

        BundleAdjustRecent();
        ReFindNewlyMade();

        if (bDoGlobal) {
            BundleAdjustAll();

            if(rand()%20 == 0)
                ReFindFromFailureQueue();

            HandleBadPoints();
        }
    }
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::StopThread()
{
    this->stop();
    this->join();
}

void MapMaker::run()
{
    assert(!mbOffline);
#ifdef WIN32
    // For some reason, I get tracker thread starvation on Win32 when
    // adding key-frames. Perhaps this will help:
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

    static gvar3<int> maxKeyFrames("MapMaker.RecentWindowSize", 4, SILENT);
    static gvar3<int> doingfullSLAM("MapMaker.fullSLAM", 0, SILENT);
    while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
    {
        if (!mbMappingEnabled) {
            boost::mutex::scoped_lock lock(MappingEnabledMut);
            MappingEnabledCond.wait(lock);
        }
        CHECK_RESET;
        sleep(5); // Sleep not really necessary, especially if mapmaker is busy
        CHECK_RESET;

        // Handle any GUI commands encountered..
        while(!mvQueuedCommands.empty())
        {
            GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
            mvQueuedCommands.erase(mvQueuedCommands.begin());
        }

        if(!mMap.IsGood())  // Nothing to do if there is no map yet!
            continue;

        // From here on, mapmaker does various map-maintenance jobs in a certain priority
        // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
        // then that takes high priority.

        CHECK_RESET;

        ros::Time timemapbegin = ros::Time::now();
        bool didba = false;
        // if the global map in the backend is updated, update the local map also
        // motion model of the tracker has to be updated correspondingly
        // read access
        boost::unique_lock< boost::mutex > lock(mSLAM.syncMutex);
        if (*doingfullSLAM && mSLAM.IsGMapUpdated()){
            // write the reference kf pose, for motion model update in tracker
            // use the second last kf as reference, since the relative pose to the last kf could be too small
            mse3LatestKFpose = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-2]->se3CfromW;
            lastKFid = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-2]->id;

            // write access
            boost::unique_lock< boost::shared_mutex > lockl(mMap.mutex);

            UpdateLMapByGMap(); // TODO adjust this when using different camera for backend
            needMotionModelUpdate = true;
            if (mSLAM.debugLoopDetected()) debugmarkLoopDetected = true;

            lockl.unlock();
            mSLAM.setGMapUpdated(false);
        }

        lock.unlock(); // end back-end lock

        // Should we run local bundle adjustment?
        // yang, also Do landing object detect here, when LBA not performan this frame, or after a certatin time gap.
        static gvar3<double> gvnPadDetectMinHeight("MapMaker.PadDetectMinHeight", 0.6, SILENT);
        if(!mbBundleConverged_Recent && (QueueSize() == 0 || nAddedKfNoBA >= *maxKeyFrames-2) && newRecentKF){
            newRecentKF=false;
            nAddedKfNoBA = 0;
            cout << "Doing LBA related staff..." << endl;

            // TODO: mark map points whose kf id had been transfored to be fixed in BA
            // chose adjusted or fixed kfs and do local BA
            cout << "Doing BA..." << endl;
            BundleAdjustRecent();
            didba = true;
            bFullBAfinished = false;
            cout<< "BA Done!"<<endl;

            // if the waiting list is not empty(might happen when very slow pgo), update the waitinglist
            // according to the current local map, for both kfs and edges
            // write access  removed in our new backend: the wl copy the same map keyframes as here
            boost::unique_lock< boost::mutex > lockwl(mSLAM.mutex_wl);
//            if (mSLAM.wlkf().size())
//                UpdateWaitingList();

            // add edges to the waiting list
            // only the front camera are used in the backend,
            // which can give a wider useful view in practice.
            // convert the keyframe type first.
            if (*doingfullSLAM){
                if (mMap.vpKeyFrames.size() > 1)
                    sendEdgesCallback(mMap.vpKeyFrames, *maxKeyFrames);//

                // update map points of the oldest kf when the kf will be deleted in next frame comes
//                if (mMap.vpKeyFramessec.size() >= *maxKeyFrames && !mMap.vpKeyFramessec[0]->pointSent){
//                    sendKfPoints(mMap.vpKeyFramessec[0]);
//                    mMap.vpKeyFramessec[0]->pointSent = true;
//                }

                // TODO: consider how to adjust kf poses for back-end when the back- and front-end share kfs
                // only do pgo after local BA? NO problem, since the back-end will only perform pgo after kfs are sent to it after BA!
                // send updated kf poses of all local kfs
//                if (mMap.vpKeyFramessec.size() >= *maxKeyFrames)
//                    updateKfPoses(mMap.vpKeyFramessec);

                // add kfs to the waiting list, add all those newly adjusted kfs
                if (!mSLAM.maxkfInied)
                    mSLAM.IniMaxkfs(*maxKeyFrames);
                for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i ++){
                    if (mMap.vpKeyFrames[i]->SentToGMap)
                        continue;

                    /// TODO: Add keyframes from multiple cameras
                    /// edges only from one camera, while kfs from multi-cam
                    sendKfCallback(mMap.vpKeyFrames[i], true);
                    mMap.vpKeyFrames[i]->SentToGMap = true;
                }
            }

            lockwl.unlock();

            CHECK_RESET;
            // do object detection?
            /*double time_now = ros::Time::now().toSec();
            static gvar3<double> gvnPadDetectFrameRate("MapMaker.PadDetectMinFrameRate", 5, SILENT);
            if (!isFinishPadDetection && (time_now - mLandingPad->time_last_detect.toSec()) > 1.0/(*gvnPadDetectFrameRate)
                    && !nullObject_keyframe &&
                    mObject_keyframe.se3CfromW.inverse().get_translation()[2]>*gvnPadDetectMinHeight){// do object detection.
                boost::unique_lock<boost::mutex>  lock(object_keyframeMut);

                // main detection function
                ComputeObjectDetectionFrame(mObject_keyframe);

                isObject_detected = false;// for debug

                lock.unlock();
                mLandingPad->time_last_detect = ros::Time::now();
                nullObject_keyframe = true;
            }*/

            cout << "Done LBA related staff." << endl;
        }
        /*else if (!isFinishPadDetection && !nullObject_keyframe &&
                 mObject_keyframe.se3CfromW.inverse().get_translation()[2]>*gvnPadDetectMinHeight){// do object detection, locate the coarse pose of the object in the map
            boost::unique_lock<boost::mutex> lock(object_keyframeMut);

            CHECK_RESET;
            // main detection function
            ComputeObjectDetectionFrame(mObject_keyframe);
            isObject_detected = false;// for debug

            lock.unlock();
            nullObject_keyframe = true;

            if (nNegative)
                ROS_INFO("POSITIVE VS NEGATIVE detection: %d, %d, %f", nPositive, nNegative, (double)nPositive/(nPositive+nNegative));
        }*/

        if (isObject_detected && !nullTracking_frame){
            boost::unique_lock<boost::mutex>  lock(tracking_frameMut);
            //          TracklandingpadESM(mTracking_frame);

            lock.unlock();
            nullTracking_frame = true;
        }

        static gvar3<int> gvnSaveFrames("MapMaker.WriteFrames", 0, SILENT);
        if (*gvnSaveFrames == 1) {
            WriteFrames("frames-ba.txt");
        }

        CHECK_RESET;
        // Are there any newly-made map points which need more measurements from older key-frames?
        if(mbBundleConverged_Recent && QueueSize() == 0)
            ReFindNewlyMade();

        CHECK_RESET;
        // Run global bundle adjustment? // drop GBA, yang.
        //      if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
        //	BundleAdjustAll();

        if (*gvnSaveFrames == 1) {
            WriteFrames("frames-ba.txt");
        }

        CHECK_RESET;
        // Very low priorty: re-find measurements marked as outliers
        if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
            ReFindFromFailureQueue();

        CHECK_RESET;
        HandleBadPoints();

        CHECK_RESET;
        // Any new key-frames to be added?
        if(QueueSize() > 0)
            AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process

        ros::Time timemapend = ros::Time::now();
        double timecost_map = timemapend.toSec() - timemapbegin.toSec();
        if (didba){
            pos_log_ << timemapbegin.toSec()<< " "
                     << timecost_map << "\n";
            // and kf size and size of map points in local map
            pos_log_ << mSLAM.keyframes_.size() << " "
                     << mMap.vpPoints.size() << "\n";
        }

        // When image input stopped, do full BA to refine the whole map, then save the full map using database tech.
        // this will be useful in specific applications
        /*ros::Duration input_interval;
        if ((imageInputCount == lastimagecount) && imageInputCount > 0){
            input_interval = ros::Time::now() - lastimageinput;
            if (input_interval.toSec() > 5){
                bInputStopped = true;
                lastimageinput = ros::Time::now();
            }
        }
        else{
            lastimagecount = imageInputCount;
            lastimageinput = ros::Time::now();
        }

        if ( (bStopForFullBA || bInputStopped) && !bFullBAfinished ) // TODO: use bstopforfullba
        {// TODO bStopForFullBA
            ROS_INFO("Doing full bundle adjustment");
            BundleAdjustAllsec();

            bFullBAfinished = true;
            mMap.didFullBA = true;
            bInputStopped = false;
            lastimageinput = ros::Time::now();
        }
        lastimagecount = imageInputCount;*/
    }
}

// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
    if (mbOffline) {
        // just do it
        this->Reset();
    } else {
        // let the mapping thread do it
        mbResetDone = false;
        mbResetRequested = true;
    }
}

bool MapMaker::ResetDone()
{
    return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
    unsigned int nBad = 0;
    // Did the tracker see this point as an outlier more often than as an inlier?
    for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
        MapPoint &p = *mMap.vpPoints[i];
        if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > 2*p.nMEstimatorInlierCount) {
            p.bBad = true;
            nBad++;
        }
    }

    // All points marked as bad will be erased - erase all records of them
    // from keyframes in which they might have been measured.

    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

    for(unsigned int i=0; i<mMap.vpPoints.size(); i++) {
        int nCam = mMap.vpPoints[i]->nSourceCamera;
        if(mMap.vpPoints[i]->bBad)
        {
            if (!nCam){
                for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++)
                {
                    KeyFrame &k = *mMap.vpKeyFrames[j];
                    if(k.mMeasurements.count(mMap.vpPoints[i]))
                        k.mMeasurements.erase(mMap.vpPoints[i]);
                }
            }
            else{
                for(unsigned int j=0; j<mMap.vpKeyFramessec[nCam - 1].size(); j++)
                {
                    KeyFrame &k = *mMap.vpKeyFramessec[nCam - 1][j];
                    if(k.mMeasurements.count(mMap.vpPoints[i]))
                        k.mMeasurements.erase(mMap.vpPoints[i]);
                }
            }
        }
    }


    // Move bad points to the trash list.
    mMap.EraseBadPoints(false);

    lock.unlock();
}

MapMaker::~MapMaker()
{
    mbBundleAbortRequested = true;

    boost::unique_lock<boost::mutex> lock(MappingEnabledMut);
    if (!mbMappingEnabled) {
        MappingEnabledCond.notify_one();
    }
    lock.unlock();

    stop(); // makes shouldStop() return true
    cout << "Waiting for mapmaker to die.." << endl;
    join();
    cout << " .. mapmaker has died." << endl;
}

// TODO: send kf and its map points seperately, with only the oldest kf's points sent now
void MapMaker::sendKfCallback(boost::shared_ptr<const ptam::KeyFrame> kf, bool sendpoints) {
//    static int lastId = -1;
    assert(kf->id == lastId++ + 1);
//    assert(kf->nSourceCamera == 1);// only send the second cam kf
    /*boost::shared_ptr<cslam::Keyframe> kfmsg(new cslam::Keyframe);

    kfmsg->id = kf->id;
    kfmsg->time = cslam::TimeStamp(kf->time.sec, kf->time.nsec);

    kfmsg->ptamPosewTc = cs_geom::toSophusSE3(kf->se3CfromW.inverse());
    kfmsg->posewTc     = kfmsg->ptamPosewTc;
    kfmsg->dSceneDepthMean = kf->dSceneDepthMean;
    kfmsg->haveGroundData = false;// never have ground measure in DCSLAM

    // copy LEVELS
    kfmsg->levels.resize(LEVELS);
    for (int i = 0; i < LEVELS; i++) {
        const ptam::Level& lev = kf->aLevels[i];
        cslam::Level& levmsg = kfmsg->levels[i];
        ImageRef irsize = lev.im.size();

        // Make sure image is actually copied:
        levmsg.image = cv::Mat(irsize[1], irsize[0], CV_8UC1,
                            (void*) lev.im.data(), lev.im.row_stride()).clone();

        levmsg.corners.resize(lev.vMaxCorners.size());

        for (uint c = 0; c < lev.vMaxCorners.size(); c++) {
            levmsg.corners[c].x = lev.vMaxCorners[c].x;
            levmsg.corners[c].y = lev.vMaxCorners[c].y;
        }
    }*/

    // Copy map points:
    /*/ by default, map points are sent only before the kf should be removed from the local map
    if (sendpoints){
        Sophus::SE3d poseInv = kfmsg->posewTc.inverse();
        for(ptam::const_meas_it it = kf->mMeasurements.begin(); it != kf->mMeasurements.end(); it++) {
            const boost::shared_ptr<ptam::MapPoint>& point = it->first;
            if (point->pPatchSourceKF.lock() &&// source kf not removed
                    !point->sourceKfIDtransfered && // avoid re-send
                    point->pPatchSourceKF.lock()->id == kf->id) {
                // point belongs to this kf
                cslam::MapPoint mpmsg;
                mpmsg.level = point->nSourceLevel;
                // p3d: PTAM stores global map points, we need relative map points:
                mpmsg.p3d = poseInv*Eigen::Vector3d(point->v3WorldPos[0], point->v3WorldPos[1], point->v3WorldPos[2]);
                const ptam::Measurement& meas = it->second;
                mpmsg.pi.x  = meas.v2RootPos[0];
                mpmsg.pi.y  = meas.v2RootPos[1];

                kfmsg->mapPoints.push_back(mpmsg);
            }
        }
    }*/

    mbackend_.addKeyframe(kf->id);
}

/*void MapMaker::sendKfPoints(boost::shared_ptr<const ptam::KeyFrame> kf) {
    assert(kf->nSourceCamera == 1);// only send the second cam kf
    boost::shared_ptr<cslam::Keyframe> kfmsg(new cslam::Keyframe);

    kfmsg->id = kf->id;// critical, set identity
    kfmsg->ptamPosewTc = cs_geom::toSophusSE3(kf->se3CfromW.inverse());
    kfmsg->posewTc     = kfmsg->ptamPosewTc;

    // Copy map points:
    // by default, map points are sent only before the kf should be removed from the local map
    Sophus::SE3d poseInv = kfmsg->posewTc.inverse();
    for(ptam::const_meas_it it = kf->mMeasurements.begin(); it != kf->mMeasurements.end(); it++) {
        const boost::shared_ptr<ptam::MapPoint>& point = it->first;
        if (point->pPatchSourceKF.lock() &&// source kf not removed
                !point->sourceKfIDtransfered && // avoid re-send
                point->pPatchSourceKF.lock()->id == kf->id) {
            // point belongs to this kf
            cslam::MapPoint mpmsg;
            mpmsg.level = point->nSourceLevel;
            // p3d: PTAM stores global map points, we need relative map points:
            mpmsg.p3d = poseInv*Eigen::Vector3d(point->v3WorldPos[0], point->v3WorldPos[1], point->v3WorldPos[2]);
            const ptam::Measurement& meas = it->second;
            mpmsg.pi.x  = meas.v2RootPos[0];
            mpmsg.pi.y  = meas.v2RootPos[1];

            kfmsg->mapPoints.push_back(mpmsg);
        }
    }

    mbackend_.addPoints(*kfmsg);
}*/

/*void MapMaker::updateKfPoses(const std::vector<boost::shared_ptr<ptam::KeyFrame> > kfs)
{
    std::vector<boost::shared_ptr<cslam::Keyframe> > kfposes;
    for (unsigned int i = 0; i < kfs.size(); i ++){
        boost::shared_ptr<ptam::KeyFrame> kf = kfs[i];
        assert(kf->nSourceCamera == 1);// only send the second cam kf

        boost::shared_ptr<cslam::Keyframe> kfmsg(new cslam::Keyframe);
        kfmsg->id = kf->id;
        kfmsg->time = cslam::TimeStamp(kf->time.sec, kf->time.nsec);

        kfmsg->ptamPosewTc = cs_geom::toSophusSE3(kf->se3CfromW.inverse());
        kfmsg->posewTc     = kfmsg->ptamPosewTc;

        kfposes.push_back(kfmsg);
    }
    mbackend_.updateKfPoses(kfposes);
}*/

// send edges to the backend
// TODO: consider tracking failure, when edge between new and failed-to-track kf should be added
void MapMaker::sendEdgesCallback(const std::vector<boost::shared_ptr<ptam::KeyFrame> > kfs, const int maxkfsize) {
    std::vector<ptam::Edge> edges;
    if (kfs.size() > 1){
        for (unsigned int i = 0; i < kfs.size() - 1; i++) {
            // aTb = aTw * wTb
            TooN::SE3<> aTb = kfs[i]->se3CfromW*kfs[i+1]->se3CfromW.inverse();
            double meanscenedepth = (kfs[i]->dSceneDepthMean + kfs[i+1]->dSceneDepthMean) / 2.0;

            ptam::Edge e;
            e.idA = kfs[i]->id;
            e.idB = kfs[i+1]->id;
            e.aTb = cs_geom::toSophusSE3(aTb);
            e.dSceneDepthMean = meanscenedepth;

            edges.push_back(e);
        }
    }

    /*/ add internal edges between the oldest kf in local BA and other NEW kfs
    if (kfs.size() >= maxkfsize){
        for (unsigned int i = kfs.size() - maxkfsize + 1; i < kfs.size(); i ++){
            TooN::SE3<> aTb = kfs[kfs.size() - maxkfsize]->se3CfromW*kfs[i]->se3CfromW.inverse();

            ptam::Edge e;
            e.idA = kfs[kfs.size() - maxkfsize]->id;
            e.idB = kfs[i]->id;
            e.aTb = cs_geom::toSophusSE3(aTb);

            edges.push_back(e);
        }
    }*/

    if (edges.size())
        mbackend_.addEdges(edges);
}

bool MapMaker::relocaliseRegister(const boost::shared_ptr<KeyFrame> goodkf, const boost::shared_ptr<KeyFrame> kf, Sophus::SE3d &result, int minInliers)
{
    if ( mbackend_.relocaliseRegister(goodkf, kf, result, minInliers))
        return true;
    else
        return false;
}

// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
    Matrix<3,4> PDash;
    PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
    PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

    Matrix<4> A;
    A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
    A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
    A[2] = v2A[0] * PDash[2] - PDash[0];
    A[3] = v2A[1] * PDash[2] - PDash[1];

    SVD<4,4> svd(A);
    Vector<4> v4Smallest = svd.get_VT()[3];
    if(v4Smallest[3] == 0.0)
        v4Smallest[3] = 0.00001;
    return project(v4Smallest);
}

// calculate the viewing angle difference from two view A and B, for a point
double MapMaker::viewAngleDiffPoint(Vector<3> vB2A, Vector<3> vB2B)
{
    double theta = 0.0;
    double costheta = vB2A*vB2B / (sqrt(vB2A*vB2A) * sqrt(vB2B*vB2B));
    assert(costheta >= 0);
    theta = acos(costheta);
    return theta;
}

bool MapMaker::InitFromRGBD(KeyFrame &kf, const TooN::SE3<> &worldPos)
{
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

    mdWiggleScale = 0.1;
    mCamera->SetImageSize(kf.aLevels[0].im.size());

    // need to copy this keyframe
    boost::shared_ptr<KeyFrame> pkFirst(new KeyFrame());
    *pkFirst = kf;
    pkFirst->SBI = kf.SBI;
    pkFirst->bFixed = true;
    pkFirst->se3CfromW = worldPos;

    // Construct map from key points
    PatchFinder finder;
    int lcount[LEVELS];
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    for(unsigned int l = 0; l < 1; l++) {
        lcount[l] = 0;
        Level& lev = kf.aLevels[l];
        const int nLevelScale = LevelScale(l);

        for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
        {
            double depth = lev.vMaxCornersDepth[i];
            // only consider corners with valid 3d position estimates
            if ((depth <= 0.0) || (depth > 4.0))
                continue;

            boost::shared_ptr<MapPoint> p(new MapPoint());

            // Patch source stuff:
            p->pPatchSourceKF = pkFirst;
            p->nSourceLevel = l;
            p->v3Normal_NC = makeVector( 0,0,-1);
            p->irCenter = lev.vMaxCorners[i];
            ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);
            p->v3Center_NC = unproject(mCamera->UnProject(irCenterl0));
            p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
            p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

            Vector<3> v3CamPos = p->v3Center_NC*depth;// Xc
            p->v3WorldPos = v3CamPos;// Xc
            p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
            p->v3RelativePos = pkFirst->se3CfromW * p->v3WorldPos;

            normalize(p->v3Center_NC);
            normalize(p->v3OneDownFromCenter_NC);
            normalize(p->v3OneRightFromCenter_NC);
            dSumDepth += depth;
            dSumDepthSquared += depth*depth;
            p->RefreshPixelVectors();
            // Do sub-pixel alignment on the same image
            finder.MakeTemplateCoarseNoWarp(*p);
            finder.MakeSubPixTemplate();
            finder.SetSubPixPos(vec(p->irCenter));
            //bool bGood = finder.IterateSubPixToConvergence(*pkFirst,10);
            //        if(!bGood)
            //          {
            //            delete p; continue;
            //          }


            mMap.vpPoints.push_back(p);

            // Construct first measurement and insert into relevant DB:
            Measurement mFirst;
            mFirst.nLevel = l;
            mFirst.Source = Measurement::SRC_ROOT;
            //            assert(l == 0);
            mFirst.v2RootPos = vec(irCenterl0); // wrt pyramid level 0
            mFirst.bSubPix = true;
            mFirst.dDepth = depth;
            pkFirst->mMeasurements[p] = mFirst;
            p->MMData.sMeasurementKFs.insert(pkFirst);
            lcount[l]++;
        }
    }

    pkFirst->dSceneDepthMean = dSumDepth / nMeas;
    pkFirst->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pkFirst->dSceneDepthMean) * (pkFirst->dSceneDepthMean));

    RefreshSceneDepth(pkFirst);
    pkFirst->id = 0;

    mMap.vpKeyFrames.push_back(pkFirst);
    mMap.bGood = true;

    lock.unlock();

    cout << "  MapMaker: made initial map from RGBD frame with " << mMap.vpPoints.size() << " points." << endl;
    cout << "map points on levels: " << lcount[0] << ", " << lcount[1] << ", " << lcount[2] << ", " << lcount[3] << std::endl;
    return true;
}

// Init the SLAM system using multiple kinects
bool MapMaker::InitFromRGBD(KeyFrame &kf, boost::shared_ptr<KeyFrame>* adkfs, const TooN::SE3<> &worldPos)
{
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

    mCamera->SetImageSize(kf.aLevels[0].im.size());
    for (int i = 0; i < AddCamNumber; i ++)
        mCameraSec[i]->SetImageSize(adkfs[i]->aLevels[0].im.size());
    int pcount[AddCamNumber];

    // need to copy this keyframe
    boost::shared_ptr<KeyFrame> pkFirst(new KeyFrame());
    *pkFirst = kf;
    pkFirst->SBI = kf.SBI;
    pkFirst->bFixed = true;
    pkFirst->se3CfromW = worldPos;

    // Construct map from key points
    PatchFinder finder;
    int lcount[LEVELS];
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    pcount[0] = 0;
    for(unsigned int l = 0; l < 1; l++) {
        lcount[l] = 0;
        Level& lev = kf.aLevels[l];
        const int nLevelScale = LevelScale(l);

        for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
        {
            double depth = lev.vMaxCornersDepth[i];
            // only consider corners with valid 3d position estimates
            if ((depth <= 0.0) || (depth > 4.0))
                continue;

            boost::shared_ptr<MapPoint> p(new MapPoint());

            // Patch source stuff:
            p->pPatchSourceKF = pkFirst;
            p->nSourceLevel = l;
            p->v3Normal_NC = makeVector( 0,0,-1);
            p->irCenter = lev.vMaxCorners[i];
            ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);
            p->v3Center_NC = unproject(mCamera->UnProject(irCenterl0));
            p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
            p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

            Vector<3> v3CamPos = p->v3Center_NC*depth;// Xc
            p->v3WorldPos = v3CamPos;// Xc
            p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
            p->v3RelativePos = pkFirst->se3CfromW * p->v3WorldPos;

            normalize(p->v3Center_NC);
            normalize(p->v3OneDownFromCenter_NC);
            normalize(p->v3OneRightFromCenter_NC);
            dSumDepth += depth;
            dSumDepthSquared += depth*depth;
            p->RefreshPixelVectors();
            // Do sub-pixel alignment on the same image
            finder.MakeTemplateCoarseNoWarp(*p);
            finder.MakeSubPixTemplate();
            finder.SetSubPixPos(vec(p->irCenter));
            //bool bGood = finder.IterateSubPixToConvergence(*pkFirst,10);
            //        if(!bGood)
            //          {
            //            delete p; continue;
            //          }


            mMap.vpPoints.push_back(p);

            // Construct first measurement and insert into relevant DB:
            Measurement mFirst;
            mFirst.nLevel = l;
            mFirst.Source = Measurement::SRC_ROOT;
            //            assert(l == 0);
            mFirst.v2RootPos = vec(irCenterl0); // wrt pyramid level 0
            mFirst.bSubPix = true;
            mFirst.dDepth = depth;
            pkFirst->mMeasurements[p] = mFirst;
            p->MMData.sMeasurementKFs.insert(pkFirst);
            lcount[l]++;
            pcount[0]++;
        }
    }

    pkFirst->dSceneDepthMean = dSumDepth / nMeas;
    pkFirst->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pkFirst->dSceneDepthMean) * (pkFirst->dSceneDepthMean));

    RefreshSceneDepth(pkFirst);
    pkFirst->id = 0;
    mMap.vpKeyFrames.push_back(pkFirst);

    // Also add kfs from those additional cameras
    for (int i = 0; i < AddCamNumber; i ++){
        boost::shared_ptr<KeyFrame> pkFirst(new KeyFrame());
        *pkFirst = *adkfs[i];
        pkFirst->SBI = kf.SBI;
        pkFirst->bFixed = true;
        pkFirst->se3CfromW = mse3Cam2FromCam1[i] * worldPos;

        // Construct map from key points
        PatchFinder finder;
        double dSumDepth = 0.0;
        double dSumDepthSquared = 0.0;
        int nMeas = 0;
        pcount[i+1] = 0;
        for(unsigned int l = 0; l < 1; l++) {
            lcount[l] = 0;
            Level& lev = kf.aLevels[l];
            const int nLevelScale = LevelScale(l);

            for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
            {
                double depth = lev.vMaxCornersDepth[i];
                // only consider corners with valid 3d position estimates
                if ((depth <= 0.0) || (depth > 4.0))
                    continue;

                boost::shared_ptr<MapPoint> p(new MapPoint());

                // Patch source stuff:
                p->pPatchSourceKF = pkFirst;
                p->nSourceLevel = l;
                p->v3Normal_NC = makeVector( 0,0,-1);
                p->irCenter = lev.vMaxCorners[i];
                ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);
                p->v3Center_NC = unproject(mCamera->UnProject(irCenterl0));
                p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
                p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

                Vector<3> v3CamPos = p->v3Center_NC*depth;// Xc
                p->v3WorldPos = v3CamPos;// Xc
                p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
                p->v3RelativePos = pkFirst->se3CfromW * p->v3WorldPos;

                normalize(p->v3Center_NC);
                normalize(p->v3OneDownFromCenter_NC);
                normalize(p->v3OneRightFromCenter_NC);
                dSumDepth += depth;
                dSumDepthSquared += depth*depth;
                p->RefreshPixelVectors();
                // Do sub-pixel alignment on the same image
                finder.MakeTemplateCoarseNoWarp(*p);
                finder.MakeSubPixTemplate();
                finder.SetSubPixPos(vec(p->irCenter));
                //bool bGood = finder.IterateSubPixToConvergence(*pkFirst,10);
                //        if(!bGood)
                //          {
                //            delete p; continue;
                //          }


                mMap.vpPoints.push_back(p);

                // Construct first measurement and insert into relevant DB:
                Measurement mFirst;
                mFirst.nLevel = l;
                mFirst.Source = Measurement::SRC_ROOT;
                //            assert(l == 0);
                mFirst.v2RootPos = vec(irCenterl0); // wrt pyramid level 0
                mFirst.bSubPix = true;
                mFirst.dDepth = depth;
                pkFirst->mMeasurements[p] = mFirst;
                p->MMData.sMeasurementKFs.insert(pkFirst);
                pcount[i+1]++;
            }
        }

        pkFirst->dSceneDepthMean = dSumDepth / nMeas;
        pkFirst->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pkFirst->dSceneDepthMean) * (pkFirst->dSceneDepthMean));

        RefreshSceneDepth(pkFirst);
        pkFirst->id = 0;
        secKFid[i] ++;

        mMap.vpKeyFramessec[i].push_back(pkFirst);
    }

    mMap.bGood = true;

    lock.unlock();

    cout << "  MapMaker: made initial map from RGBD frame with " << mMap.vpPoints.size() << " points." << endl;
    cout << "map points from each camera: " << pcount[0] << ", " ;
    for (int i = 0; i < AddCamNumber; i ++)
        cout << pcount[i] << ", ";
    cout << std::endl;

    return true;
}

// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the 
bool MapMaker::InitFromStereo(KeyFrame &kF,
                              KeyFrame &kS,
                              vector<pair<ImageRef, ImageRef> > &vTrailMatches,
                              SE3<> &se3TrackerPose,
                              bool use_circle_ini, int ini_thresh, int ini_times)
{

    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

    mdWiggleScale = *mgvdWiggleScale; // Cache this for the new map.

    mCamera->SetImageSize(kF.aLevels[0].im.size());

    vector<HomographyMatch> vMatches;
    for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
        HomographyMatch m;
        m.v2CamPlaneFirst = mCamera->UnProject(vTrailMatches[i].first);
        m.v2CamPlaneSecond = mCamera->UnProject(vTrailMatches[i].second);
        m.m2PixelProjectionJac = mCamera->GetProjectionDerivs();
        vMatches.push_back(m);
    }

    SE3<> se3;
    bool bGood;

    HomographyInit HomographyInit;
    bGood = HomographyInit.Compute(vMatches, 5.0, se3);//R21
    if(!bGood)
    {
        cout << "  Could not init from stereo pair, try again." << endl;
        return false;
    }

    // Check that the initialiser estimated a non-zero baseline
    double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
    if(dTransMagn == 0)
    {
        cout << "  Estimated zero baseline from stereo pair, try again." << endl;
        return false;
    }
    // change the scale of the map so the second camera is wiggleScale away from the first
    if (use_circle_ini)
    {
        bGood = HomographyInit.TransIniFromCircle(kF.se3CfromW, kS.se3CfromW, 0.2, mdWiggleScale);
        if(!bGood)
        {
            cout << "  Could not init from stereo pair, try again." << endl;
            return false;
        }
        else if (ini_times < ini_thresh)
        {
            cout << "  Begin to init from stereo pair, wait for next frame." << endl;
            return true;
        }
    }
    se3.get_translation() *= mdWiggleScale/dTransMagn;

    boost::shared_ptr<KeyFrame> pkFirst(new KeyFrame());
    boost::shared_ptr<KeyFrame> pkSecond(new KeyFrame());
    *pkFirst = kF;
    *pkSecond = kS;

    pkFirst->bFixed = true;
    SE3<> se3first;
    if (use_circle_ini)
        se3first = pkFirst->se3CfromW;//R1w
    pkFirst->se3CfromW = SE3<>();

    pkSecond->bFixed = false;
    pkSecond->se3CfromW = se3;//R21

    // Construct map from the stereo matches.
    PatchFinder finder;

    for(unsigned int i=0; i<vMatches.size(); i++)
    {
        boost::shared_ptr<MapPoint> p(new MapPoint());

        // Patch source stuff:
        p->pPatchSourceKF = pkFirst;
        p->nSourceLevel = 0;
        p->v3Normal_NC = makeVector( 0,0,-1);
        p->irCenter = vTrailMatches[i].first;
        p->v3Center_NC = unproject(mCamera->UnProject(p->irCenter));
        p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(p->irCenter + ImageRef(0,1)));
        p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(p->irCenter + ImageRef(1,0)));
        normalize(p->v3Center_NC);
        normalize(p->v3OneDownFromCenter_NC);
        normalize(p->v3OneRightFromCenter_NC);
        p->RefreshPixelVectors();

        // Do sub-pixel alignment on the second image
        finder.MakeTemplateCoarseNoWarp(*p);
        finder.MakeSubPixTemplate();
        finder.SetSubPixPos(vec(vTrailMatches[i].second));
        bool bGood = finder.IterateSubPixToConvergence(*pkSecond,10);
        if(!bGood)
            continue;

        // Triangulate point:
        Vector<2> v2SecondPos = finder.GetSubPixPos();
        p->v3WorldPos = ReprojectPoint(se3, mCamera->UnProject(v2SecondPos), vMatches[i].v2CamPlaneFirst);
        if(p->v3WorldPos[2] < 0.0)
            continue;
        p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
        p->v3RelativePos = pkFirst->se3CfromW * p->v3WorldPos;

        // Not behind map? Good, then add to map.
        mMap.vpPoints.push_back(p);

        // Construct first two measurements and insert into relevant DBs:
        Measurement mFirst;
        mFirst.nLevel = 0;
        mFirst.Source = Measurement::SRC_ROOT;
        mFirst.v2RootPos = vec(vTrailMatches[i].first);
        mFirst.bSubPix = true;
        mFirst.dDepth = 0.0;
        pkFirst->mMeasurements[p] = mFirst;
        p->MMData.sMeasurementKFs.insert(pkFirst);

        Measurement mSecond;
        mSecond.nLevel = 0;
        mSecond.Source = Measurement::SRC_TRAIL;
        mSecond.v2RootPos = finder.GetSubPixPos();
        mSecond.bSubPix = true;
        mSecond.dDepth = 0.0;
        pkSecond->mMeasurements[p] = mSecond;
        p->MMData.sMeasurementKFs.insert(pkSecond);
    }

    mMap.vpKeyFrames.push_back(pkFirst);
    mMap.vpKeyFrames.push_back(pkSecond);
    pkFirst->MakeKeyFrame_Rest();
    pkSecond->MakeKeyFrame_Rest();

    for(int i=0; i<5; i++)
        BundleAdjustAll();

    // Estimate the feature depth distribution in the first two key-frames
    // (Needed for epipolar search)
    RefreshSceneDepth(pkFirst);
    RefreshSceneDepth(pkSecond);
    mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;


    AddSomeMapPoints(0);
    AddSomeMapPoints(3);
    AddSomeMapPoints(1);
    AddSomeMapPoints(2);

    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;

    while(!mbBundleConverged_Full)
    {
        BundleAdjustAll();
        if(mbResetRequested)
            return false;
    }

    // Rotate and translate the map so the dominant plane is at z=0:
    SE3<> se3worldfromfirst = SE3<>();//RW1
    if (!use_circle_ini)
        se3worldfromfirst = CalcPlaneAligner();
    else{
        //      se3worldfromfirst = CalcPlaneAligner();//RTw1, with yaw angle ambiguity
        //      //retrieve yaw angle from the letter H.

        //      se3worldfromfirst.get_translation() = se3first.inverse().get_translation();//Tw1
        se3worldfromfirst = se3first.inverse();//set the world frame coincide with circle pattern
    }

    ApplyGlobalTransformationToMap(se3worldfromfirst);
    mMap.bGood = true;

    if (!use_circle_ini)
        se3TrackerPose = pkSecond->se3CfromW;//RT2W
    else
        se3TrackerPose = pkSecond->se3CfromW;// * se3worldfromfirst.inverse();//RT2W = RT21 * RT1W

    lock.unlock();

    cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;
    return true;
}

// InitFromOneCircle() generates the initial map points from one keyframes with a known circle in it
// and a vector of image correspondences. Uses the
//ini_thresh is the minimal number of image features in the first keyframe
bool MapMaker::InitFromOneCircle(KeyFrame &kf,
                                 SE3<> se3TrackerPose,
                                 int ini_thresh, int ini_times)
{

    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

    mdWiggleScale = 0.5;
    //first, there could introuduce some restriction to the pose of the quadrotor
    //the minimal number of the features which have a reasonable score should be restricted
    int numCorners = 0;
    for (int i = 0; i <LEVELS; i ++){// tricky, only use middle two level features for ini
        //        if ((i == 1) || (i == 2))
        //                continue;
        numCorners += kf.aLevels[i].vMaxCorners.size();
    }
    if (numCorners < ini_thresh){//good features of the lowest level
        cout << "  Could not init from this frame, too few maximal features: " << numCorners << endl;
        return false;
    }

    //calculate the 3D position of the good features, assumed to be located on the circle plane
    //different from using RGBD sensor, we need to estimate the pose of those feature points by
    //assuming the lay on the same plane of the circle.
    //add all these 3D point features to the new map
    mCamera->SetImageSize(kf.aLevels[0].im.size());
    // need to copy this keyframe
    boost::shared_ptr<KeyFrame> pkFirst(new KeyFrame());
    *pkFirst = kf;
    pkFirst->SBI = kf.SBI;
    pkFirst->bFixed = true;
    pkFirst->se3CfromW = se3TrackerPose;

    // Construct map from key points
    PatchFinder finder;
    int lcount[LEVELS];
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    for(unsigned int l = 0; l < LEVELS; l++) {
        lcount[l] = 0;
        Level& lev = kf.aLevels[l];
        const int nLevelScale = LevelScale(l);
        //        if ((l == 0)||(l == 3))// ignor 0 and 3 level features for map
        //        if ((l == 0))// only use high level features when ini with ground/circle
        //            continue;
        // randomly chose some points to be added to the map.
        int pickpart = 1;// chose 1/4 of all corners
        int levelpointnum = lev.vMaxCorners.size();
        vector<int> cornerpicked;
        for (int i = 0; i < levelpointnum; i ++)
            cornerpicked.push_back(i);
        random_shuffle(cornerpicked.begin(), cornerpicked.end());
        cornerpicked.resize(levelpointnum/pickpart);
        for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
        {
            bool parthasi = false;
            for (int partnum = 0; partnum < cornerpicked.size(); partnum ++){
                if (cornerpicked[partnum] == i)
                    parthasi = true;
            }
//            if (!parthasi) continue;

            boost::shared_ptr<MapPoint> p(new MapPoint());

            // Patch source stuff:
            p->pPatchSourceKF = pkFirst;
            p->nSourceLevel = l;
            p->v3Normal_NC = makeVector( 0,0,-1);
            p->irCenter = lev.vMaxCorners[i];
            ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);

            // for paper, hovering on different object, ignore those object want to detect
            if (false){
                if (irCenterl0.x < 300 && irCenterl0.y > 365)
                    continue;
                else if (irCenterl0.y < 210 && irCenterl0.x > 520)
                    continue;
            }

            p->v3Center_NC = unproject(mCamera->UnProject(irCenterl0));// (x, y, 1)
            p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
            p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

            //calculate the 3D point info in world frame. xp--(xd--)xn(--xc)--xw
            //Xw = Rwc*Xc + twc = d*Rwc*Xn + twc
            //zw = Xw[2] = 0 --> d = -twc[2]/(Rwc(2,:)*Xn)
            //-->xw, yw, zw

            double depth = se3TrackerPose.inverse().get_rotation().get_matrix()[2]*p->v3Center_NC;
            if (depth!=0.0)
                depth = - se3TrackerPose.inverse().get_translation()[2] / depth;
            else
                continue;
            if(depth <= 0.0)
                continue;
            double xw, yw;

            xw = se3TrackerPose.inverse().get_translation()[0] + se3TrackerPose.inverse().get_rotation().get_matrix()[0]*p->v3Center_NC*depth;
            yw = se3TrackerPose.inverse().get_translation()[1] + se3TrackerPose.inverse().get_rotation().get_matrix()[1]*p->v3Center_NC*depth;

            p->v3WorldPos = makeVector(xw, yw, 0.0);
            p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
            p->v3RelativePos = pkFirst->se3CfromW * p->v3WorldPos;

            //in full slam, also keep its relative pose to source kf
//            p->v3SourceKFfromeWorld = se3TrackerPose;
//            p->v3RelativePos = se3TrackerPose * p->v3WorldPos;

            dSumDepth += depth;
            dSumDepthSquared += depth*depth;
            nMeas ++;
            p->RefreshPixelVectors();

            normalize(p->v3Center_NC);
            normalize(p->v3OneDownFromCenter_NC);
            normalize(p->v3OneRightFromCenter_NC);

            // Do sub-pixel alignment on the same image
            finder.MakeTemplateCoarseNoWarp(*p);
            finder.MakeSubPixTemplate();
            finder.SetSubPixPos(vec(p->irCenter));

            mMap.vpPoints.push_back(p);

            // Construct first measurement and insert into relevant DB:
            Measurement mFirst;
            mFirst.nLevel = l;
            mFirst.Source = Measurement::SRC_ROOT;
            mFirst.v2RootPos = vec(irCenterl0); // wrt pyramid level 0
            mFirst.bSubPix = true;
            mFirst.dDepth = depth;
            pkFirst->mMeasurements[p] = mFirst;
            p->MMData.sMeasurementKFs.insert(pkFirst);
            lcount[l]++;
        }
    }
    //    cout << "valid corners: " << lcount[0]+lcount[1] + lcount[2] + lcount[3] << endl;
    cout << "valid corners: " << lcount[1] + lcount[2]<< endl;
    pkFirst->dSceneDepthMean = dSumDepth / nMeas;
    pkFirst->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pkFirst->dSceneDepthMean) * (pkFirst->dSceneDepthMean));

    RefreshSceneDepth(pkFirst);
    pkFirst->id = 0;

    mMap.vpKeyFrames.push_back(pkFirst);
    mMap.bGood = true;

    lock.unlock();

    cout << "  MapMaker: made initial map from " << mMap.vpPoints.size() << " points." << endl;
    cout << "map points on levels: " << nMeas << std::endl;

//    mMap.baDoneCallback(mMap.vpKeyFrames);
    return true;
}
bool MapMaker::InitFromOneCircleDual(KeyFrame &kf, KeyFrame &kss,
                                 SE3<> se3TrackerPose,
                                 int ini_thresh, int ini_times)
{

    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

    mdWiggleScale = 0.5;
    //first, there could introuduce some restriction to the pose of the quadrotor
    //the minimal number of the features which have a reasonable score should be restricted
    int numCorners = 0;
    for (int i = 0; i <LEVELS; i ++){// tricky, only use middle two level features for ini
        //        if ((i == 1) || (i == 2))
        //                continue;
        numCorners += kf.aLevels[i].vMaxCorners.size();
    }
    if (numCorners < ini_thresh){//good features of the lowest level
        cout << "  Could not init from this frame, too few maximal features: " << numCorners << endl;
        return false;
    }

    //calculate the 3D position of the good features, assumed to be located on the circle plane
    //different from using RGBD sensor, we need to estimate the pose of those feature points by
    //assuming the lay on the same plane of the circle.
    //add all these 3D point features to the new map
    mCamera->SetImageSize(kf.aLevels[0].im.size());
    // need to copy this keyframe
    boost::shared_ptr<KeyFrame> pkFirst(new KeyFrame());
    *pkFirst = kf;
    pkFirst->SBI = kf.SBI;
    pkFirst->bFixed = true;
    pkFirst->se3CfromW = se3TrackerPose;

    // Construct map from key points
    PatchFinder finder;
    int lcount[LEVELS];
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    for(unsigned int l = 0; l < LEVELS; l++) {
        lcount[l] = 0;
        Level& lev = kf.aLevels[l];
        const int nLevelScale = LevelScale(l);
        //        if ((l == 0)||(l == 3))// ignor 0 and 3 level features for map
        //        if ((l == 0))// only use high level features when ini with ground/circle
        //            continue;
        for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
        {
            boost::shared_ptr<MapPoint> p(new MapPoint());

            // Patch source stuff:
            p->pPatchSourceKF = pkFirst;
            p->nSourceLevel = l;
            p->v3Normal_NC = makeVector( 0,0,-1);
            p->irCenter = lev.vMaxCorners[i];
            ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);

            // for paper, hovering on different object, ignore those object want to detect
            if (false){
                if (irCenterl0.x < 300 && irCenterl0.y > 365)
                    continue;
                else if (irCenterl0.y < 210 && irCenterl0.x > 520)
                    continue;
            }

            p->v3Center_NC = unproject(mCamera->UnProject(irCenterl0));// (x, y, 1)
            p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
            p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

            //calculate the 3D point info in world frame. xp--(xd--)xn(--xc)--xw
            //Xw = Rwc*Xc + twc = d*Rwc*Xn + twc
            //zw = Xw[2] = 0 --> d = -twc[2]/(Rwc(2,:)*Xn)
            //-->xw, yw, zw

            double depth = se3TrackerPose.inverse().get_rotation().get_matrix()[2]*p->v3Center_NC;
            if (depth!=0.0)
                depth = - se3TrackerPose.inverse().get_translation()[2] / depth;
            else
                continue;
            if(depth <= 0.0)
                continue;
            double xw, yw;

            xw = se3TrackerPose.inverse().get_translation()[0] + se3TrackerPose.inverse().get_rotation().get_matrix()[0]*p->v3Center_NC*depth;
            yw = se3TrackerPose.inverse().get_translation()[1] + se3TrackerPose.inverse().get_rotation().get_matrix()[1]*p->v3Center_NC*depth;

            p->v3WorldPos = makeVector(xw, yw, 0.0);
            p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
            p->v3RelativePos = pkFirst->se3CfromW * p->v3WorldPos;

            dSumDepth += depth;
            dSumDepthSquared += depth*depth;
            nMeas ++;
            p->RefreshPixelVectors();

            normalize(p->v3Center_NC);
            normalize(p->v3OneDownFromCenter_NC);
            normalize(p->v3OneRightFromCenter_NC);

            // Do sub-pixel alignment on the same image
            finder.MakeTemplateCoarseNoWarp(*p);
            finder.MakeSubPixTemplate();
            finder.SetSubPixPos(vec(p->irCenter));

            mMap.vpPoints.push_back(p);

            // Construct first measurement and insert into relevant DB:
            Measurement mFirst;
            mFirst.nLevel = l;
            mFirst.Source = Measurement::SRC_ROOT;
            mFirst.v2RootPos = vec(irCenterl0); // wrt pyramid level 0
            mFirst.bSubPix = true;
            mFirst.dDepth = depth;
            pkFirst->mMeasurements[p] = mFirst;
            p->MMData.sMeasurementKFs.insert(pkFirst);
            lcount[l]++;
        }
    }
    //    cout << "valid corners: " << lcount[0]+lcount[1] + lcount[2] + lcount[3] << endl;
    cout << "valid corners: " << lcount[1] + lcount[2]<< endl;
    pkFirst->dSceneDepthMean = dSumDepth / nMeas;
    pkFirst->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pkFirst->dSceneDepthMean) * (pkFirst->dSceneDepthMean));

    RefreshSceneDepth(pkFirst);

    pkFirst->id = 0;

    mMap.vpKeyFrames.push_back(pkFirst);
    mMap.bGood = true;

    lock.unlock();

    cout << "  MapMaker: made initial map from " << mMap.vpPoints.size() << " points." << endl;
    cout << "map points on levels: " << nMeas << std::endl;

    // add keyframe for the second cam
    boost::shared_ptr<KeyFrame> pkSec(new KeyFrame());
    *pkSec = kss;
    pkSec->SBI = SmallBlurryImage(*pkSec); // Regenerate Small Blurry Image
    pkSec->SBI.MakeJacs();
    if (!pkSec->bComplete)
        pkSec->MakeKeyFrame_Rest();
    pkSec->bFixed = true;
    pkSec->mAssociateKeyframe = false;
    pkSec->se3CfromW = mse3Cam2FromCam1[0] * se3TrackerPose;
    pkSec->id = 0;

    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > locksec(mMap.mutex);
    mMap.vpKeyFramessec[0].push_back(pkSec);
//    CVD::img_save(pK->aLevels[0].im, "vpKeyFramessec.jpg");
    // Any measurements? Update the relevant point's measurement counter status map
//    for(meas_it it = pkSec->mMeasurements.begin();
//        it!=pkSec->mMeasurements.end();
//        it++)
//    {
//        it->first->MMData.sMeasurementKFs.insert(pkSec);
//        it->second.Source = Measurement::SRC_TRACKER;
//    }

    locksec.unlock();

    return true;
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt 
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel)
{
    vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
    vector<Candidate> vCGood;
    vector<ImageRef> irBusyLevelPos;
    // Make a list of `busy' image locations, which already have features at the same level
    // or at one level higher.
    for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
    {
        if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
            continue;
        irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
    }

    // Only keep those candidates further than 10 pixels away from busy positions.
    unsigned int nMinMagSquared = 10*10;
    for(unsigned int i=0; i<vCSrc.size(); i++)
    {
        ImageRef irC = vCSrc[i].irLevelPos;
        bool bGood = true;
        for(unsigned int j=0; j<irBusyLevelPos.size(); j++)
        {
            ImageRef irB = irBusyLevelPos[j];
            if((irB - irC).mag_squared() < nMinMagSquared)
            {
                bGood = false;
                break;
            }
        }
        if(bGood)
            vCGood.push_back(vCSrc[i]);
    }
    vCSrc = vCGood;
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel, int nCam)
{
    boost::shared_ptr<KeyFrame> kSrc;
    boost::shared_ptr<KeyFrame> kTarget;
    if (!nCam)
    {
        kSrc = mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]; // The new keyframe
        kTarget = ClosestKeyFrame(kSrc, 0.15);
    }
    else
    {
        kSrc = mMap.vpKeyFramessec[nCam - 1][mMap.vpKeyFramessec[nCam - 1].size() - 1];
        kTarget = ClosestKeyFrame(kSrc,0.1);
    }
    static gvar3<double> gvnmindistxy("MapMaker.mindistxy", 0.1, SILENT);
    SE3<> c12 = kSrc->se3CfromW * kTarget->se3CfromW.inverse();
    double distxy = sqrt(c12.get_translation()[0]*c12.get_translation()[0]+c12.get_translation()[1]*c12.get_translation()[1]);
    if (nCam && distxy < *gvnmindistxy)
        return;
    Level &l = kSrc->aLevels[nLevel];

    ThinCandidates(*kSrc, nLevel);

    static gvar3<int> gvnAddPoints3D("MapMaker.AddPoints3D", 1, SILENT);
    static gvar3<int> gvnAddPointsEpipolar("MapMaker.AddPointsEpipolar", 1, SILENT);

    //  static int add = 0;
    for(unsigned int i = 0; i<l.vCandidates.size(); i++) {
        if (l.vCandidates[i].dDepth > 0.0 && *gvnAddPoints3D == 1){
            AddPointDepth(kSrc, kTarget, nLevel, i);
            //          if (add == 0)
            //              cout << "ADD MAP POINTS BY DEPTH. " << endl;
            //          add = 1;
        }
        else if (*gvnAddPointsEpipolar == 1){
            AddPointEpipolar(kSrc, kTarget, nLevel, i);
            //          if (add == 0)
            //            cout << "ADD MAP POINTS BY EPIPOLAR SEARCH. " << endl;
            //          add = 1;
        }
    }
    //  add = 0;
};

// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();

    for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
        mMap.vpPoints[i]->v3WorldPos =
                se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
        mMap.vpPoints[i]->RefreshPixelVectors();
    }
}

// Applies a global scale factor to the map
void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;

    for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
        (*mMap.vpPoints[i]).v3WorldPos *= dScale;
        (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
        (*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
        (*mMap.vpPoints[i]).RefreshPixelVectors();
    }
}

bool MapMaker::AddObjectDetectionFrame(KeyFrame &k)
{
    boost::unique_lock<boost::mutex> lock(object_keyframeMut);

    mObject_keyframe = k;

    lock.unlock();
    nullObject_keyframe = false;

    return true;
}

bool MapMaker::AddPadTrackingFrame(KeyFrame &k)
{
    boost::unique_lock<boost::mutex>  lock(tracking_frameMut);
    mTracking_frame = k;

    lock.unlock();
    nullTracking_frame = false;

    return true;
}

void MapMaker::ComputeObjectDetectionFrame(KeyFrame &k)
{
    ros::Time detectb = ros::Time::now();
    // with descriptors of the ref imgs got, compute descriptors of current frame levels.im
    for (int i = 0; i < objLEVEL; i ++){
        if (i > 1) // only try just the 0 and 1 level.
            continue;

        // retrieve opencv img
        cv::Mat cur_img(k.aLevels[i].im.size().y, k.aLevels[i].im.size().x, CV_8UC1, k.aLevels[i].im.data());
        // retrieve opencv keypoints.
        // TODO: how to make this more efficient.
        vector<cv::KeyPoint> points;
        cv::KeyPoint point;
        for (unsigned int j = 0; j < k.aLevels[i].vMaxCorners.size(); j ++){
            point.pt.x = k.aLevels[i].vMaxCorners[j].x;
            point.pt.y = k.aLevels[i].vMaxCorners[j].y;
            point.size = 15;
            point.octave = i;
            points.push_back(point);
        }
        //        mLandingPad->padfeature_detect(cur_img, points);

        // compute opencv descriptors
        cv::Mat descriptors;
        mLandingPad->currentfeature_descript(cur_img, points, descriptors);// those points cannot be descript will be remove, in opencv function.

        mLandingPad->load_curcorners(points, i);
        mLandingPad->load_curdescriptors(descriptors, i);
        mLandingPad->load_curimgs(cur_img, i);

        if(i == 0)
            mLandingPad->ini_allcurcorners(points);
        if (i > 0){
            for (int j = 0; j < points.size(); j ++)
                mLandingPad->add_allcurcorners(points[j]);
        }
    }
    mLandingPad->load_allcurdescriptors();

    ROS_INFO("CUR FEATURE AND DESCRIPTOR NUMS: %d %d", mLandingPad->GetallCurpoints().size(), mLandingPad->GetallCurrentDescriptors().rows);

    ros::Duration detect = ros::Time::now() - detectb;
    ros::Time matchtimeb = ros::Time::now();

    // match features, get those good matches
    // features of different refimg levels could be matched to the corresponding currentimg level or lower levels
    // refimg levels          currentimg levels
    //       1        --> (1)           1
    //       2        --> (1 2)         2
    //       3        --> (1, 2, 3)     3
    /*/ but, try just match to the 0 level.
    for (int i = 0; i < objLEVEL; i ++){
        if (i < 1)
            continue;
        if (mLandingPad->GetRefDescriptors(i).rows && mLandingPad->GetCurrentDescriptors(0).rows){ // descriptors not null

            vector<vector<cv::DMatch> > matchesvec;
            vector<cv::DMatch> matches;
            mLandingPad->padfeature_match(mLandingPad->GetRefDescriptors(i), mLandingPad->GetCurrentDescriptors(0),
                                          matchesvec);
            for (int j = 0; j < matchesvec.size(); j ++){
                if (matchesvec[j].size()>=2){
                    if (matchesvec[j][0].distance/matchesvec[j][1].distance < 0.8)
                    {// ratio is important
                        matches.push_back(matchesvec[j][0]);
                    }
                }
                else if (matchesvec[i].size() == 1)
                    matches.push_back(matchesvec[i][0]);
            }
//            mLandingPad->padfeature_match(mLandingPad->GetRefDescriptors(i), mLandingPad->GetCurrentDescriptors(0),
//                                          matches);
//            mLandingPad->load_matches(matches, i);

            // then find a good match
//            double max_dist = 0;
            double min_dist = 9999999999999999.9;
            for (unsigned int j = 0; j < matches.size(); j ++){
                double dist =  matches[j].distance;
                if( dist < min_dist ) min_dist = dist;
//                if( dist > max_dist ) max_dist = dist;
            }
            vector< cv::DMatch > good_matches, bad_matches;
            for(unsigned int j = 0; j < matches.size(); j++ )
            {
                cv::DMatch match = matches[j];
                if(match.distance < 3*min_dist )
                    good_matches.push_back( match);
//                else
//                    bad_matches.push_back( match);
            }
            mLandingPad->load_matches(good_matches, i);
            ROS_INFO("GOOD MATCH NUM: %d", good_matches.size());
        }
        else
            ROS_INFO("NO MATCHES FOUND IN LEVEL %d!", i);
    }//*/
    // match all features on all levels
    // free match among ref 1-2 with cur 0-1
    if (mLandingPad->GetallRefDescriptors().rows && mLandingPad->GetallCurrentDescriptors().rows){ // descriptors not null

        vector<vector<cv::DMatch> > matchesvec;
        vector<cv::DMatch> matches;
        vector<cv::DMatch> matches11;
        vector<cv::DMatch> matches12;
        vector<cv::DMatch> matches21;
        vector<cv::DMatch> matches22;
        mLandingPad->padfeature_match(mLandingPad->GetallRefDescriptors(), mLandingPad->GetallCurrentDescriptors(),
                                      matchesvec);
        for (int i = 0; i < matchesvec.size(); i ++){
            if (matchesvec[i].size()>=2){
                if (matchesvec[i][0].distance/matchesvec[i][1].distance < 0.8)
                {// ratio is important
                    matches.push_back(matchesvec[i][0]);
                }
            }
            else if (matchesvec[i].size() == 1)
                matches.push_back(matchesvec[i][0]);
        }
        // then find a good match
        //            double max_dist = 0;
        double min_dist = 9999999999999999.9;
        for (unsigned int j = 0; j < matches.size(); j ++){
            double dist =  matches[j].distance;
            if( dist < min_dist ) min_dist = dist;
            //                if( dist > max_dist ) max_dist = dist;
        }
        vector< cv::DMatch > good_matches, bad_matches;
        for(unsigned int j = 0; j < matches.size(); j++ )
        {
            cv::DMatch match = matches[j];
            if(match.distance < 5*min_dist ){
                good_matches.push_back( match);
                switch (mLandingPad->GetallRefpoints()[match.queryIdx].octave){
                case 1:
                    if (mLandingPad->GetallCurpoints()[match.trainIdx].octave == 0)
                        matches11.push_back(match);
                    else if (mLandingPad->GetallCurpoints()[match.trainIdx].octave == 1)
                        matches12.push_back(match);
                    break;
                case 2:
                    if (mLandingPad->GetallCurpoints()[match.trainIdx].octave == 0)
                        matches21.push_back(match);
                    else if (mLandingPad->GetallCurpoints()[match.trainIdx].octave == 1)
                        matches22.push_back(match);
                    break;
                default:break;
                }
            }
            //                else
            //                    bad_matches.push_back( match);
        }
        mLandingPad->load_matches_ful(good_matches);
        mLandingPad->load_matches(matches11, 0);
        mLandingPad->load_matches(matches12, 1);
        mLandingPad->load_matches(matches21, 2);
        mLandingPad->load_matches(matches22, 3);
        ROS_INFO("ALL AND GOOD MATCH NUM: %d, %d, %d", matchesvec.size(), matches.size(), good_matches.size());
    }
    else
        ROS_INFO("NO MATCHES FOUND!");//*/

    /*/ match refimg ORB features on all levels
    // free match among ref ORB with cur 0-1 myORB
    static gvar3<double> gvnGoodMatchRatio("MapMaker.GoodMatchRatio", 0.8, SILENT);
    if (mLandingPad->GetallRefDescriptors().rows && mLandingPad->GetallCurrentDescriptors().rows){ // descriptors not null

        vector<vector<cv::DMatch> > matchesvec;
        vector<cv::DMatch> matches;
        vector<cv::DMatch> matches11;
        vector<cv::DMatch> matches12;
        mLandingPad->padfeature_match(mLandingPad->GetallRefDescriptors(), mLandingPad->GetallCurrentDescriptors(),
                                      matchesvec);
        for (int i = 0; i < matchesvec.size(); i ++){
            if (matchesvec[i].size()>=2){
                if (matchesvec[i][0].distance/matchesvec[i][1].distance < *gvnGoodMatchRatio)
                {// ratio is important
                    matches.push_back(matchesvec[i][0]);
                }
            }
            else if (matchesvec[i].size() == 1)
                matches.push_back(matchesvec[i][0]);
        }
        // then find a good match
//            double max_dist = 0;
        double min_dist = 9999999999999999.9;
        for (unsigned int j = 0; j < matches.size(); j ++){
            double dist =  matches[j].distance;
            if( dist < min_dist ) min_dist = dist;
//                if( dist > max_dist ) max_dist = dist;
        }
        vector< cv::DMatch > good_matches, bad_matches;
        for(unsigned int j = 0; j < matches.size(); j++ )
        {
            cv::DMatch match = matches[j];
            if(match.distance < 5*min_dist ){
                good_matches.push_back( match);
                switch (mLandingPad->GetallCurpoints()[match.trainIdx].octave){
                case 0:
                    matches11.push_back(match);
                    break;
                case 1:
                    matches12.push_back(match);
                    break;
                default:break;
                }
            }
//                else
//                    bad_matches.push_back( match);
        }
        mLandingPad->load_matches_ful(good_matches);
        mLandingPad->load_matches(matches11, 0);
        mLandingPad->load_matches(matches12, 1);
        ROS_INFO("ALL AND GOOD MATCH NUM: %d, %d, %d", matchesvec.size(), matches.size(), good_matches.size());
    }
    else
        ROS_INFO("NO MATCHES FOUND!");//*/


    ros::Duration matchtime = ros::Time::now() - matchtimeb;
    ROS_INFO("Descript and matching time: %f, %f", detect.toSec(), matchtime.toSec());

    /*/ get those rectified matched image points
    vector<cv::Point2f> refframe;
    vector<cv::Point2f> curframe;
    ros::Time homotimeb = ros::Time::now();
    for (int i = 0; i < objLEVEL; i ++){
        if (i < 1)// debug using matches from specified level
            continue;

        if (mLandingPad->GetMatches(i).size() > 8){
            cv::Point2f obj, cur;
            for( int j = 0; j < mLandingPad->GetMatches(i).size(); j++ )
            {
                //-- Get the keypoints from the good matches
                // ref image should have been undistorted already
                obj = mLandingPad->GetRefpoints(i)[mLandingPad->GetMatches(i)[j].queryIdx].pt;
                // project the current frame points to undistorted image frame
                cur = mLandingPad->GetCurpoints(0)[mLandingPad->GetMatches(i)[j].trainIdx].pt;
                TooN::Vector<2> undistim;
                undistim[0] = cur.x;
                undistim[1] = cur.y;
                undistim = mCamera->UnProject(undistim);
                undistim = mCamera->Project_ud(undistim);
                cur.x = undistim[0];
                cur.y = undistim[1];
                curframe.push_back( cur );
                // project matches in different level to the first level, so we get one consistant match.
                switch(i) {
                case 0: refframe.push_back(obj); break;
                default: {
                    obj.x = obj.x * (1 << i);
                    obj.y = obj.y * (1 << i);
                    refframe.push_back(obj);
                    break;
                }
                }
            }
        }
        else
            ROS_INFO("TOO FEW MATCHES ON LEVEL %d !", i);
    }//*/
    // get those rectified matched image points from all levels
    vector<cv::Point2f> refframe;
    vector<cv::Point2f> curframe;
    ros::Time homotimeb = ros::Time::now();
    static gvar3<int> gvnRefORBfirstLevel("MapMaker.RefORBfirstLevel", 1, SILENT);
    if (mLandingPad->GetMatches_ful().size() > 8){
        cv::KeyPoint obj, cur;
        for( int j = 0; j < mLandingPad->GetMatches_ful().size(); j++ )
        {
            //-- Get the keypoints from the good matches
            // ref image should have been undistorted already
            obj = mLandingPad->GetallRefpoints()[mLandingPad->GetMatches_ful()[j].queryIdx];
            // project the current frame points to undistorted image frame
            cur = mLandingPad->GetallCurpoints()[mLandingPad->GetMatches_ful()[j].trainIdx];
            // project matches in different level to the first level, so we get one consistant match.
            if (obj.octave < *gvnRefORBfirstLevel)
                continue;
            switch(cur.octave) {
            case 0: break;
            default: {
                cur.pt.x = cur.pt.x * (1 << cur.octave);
                cur.pt.y = cur.pt.y * (1 << cur.octave);
                break;
            }
            }
            TooN::Vector<2> undistim;
            undistim[0] = cur.pt.x;
            undistim[1] = cur.pt.y;
            undistim = mCamera->UnProject(undistim);
            undistim = mCamera->Project_ud(undistim);
            cur.pt.x = undistim[0];
            cur.pt.y = undistim[1];
            curframe.push_back( cur.pt );
            // project matches in different level to the first level, so we get one consistant match.
            switch(obj.octave) {
            case 0: refframe.push_back(obj.pt); break;
            default: {
                obj.pt.x = obj.pt.x * (1 << obj.octave);
                obj.pt.y = obj.pt.y * (1 << obj.octave);
                refframe.push_back(obj.pt);
                break;
            }
            }
        }
    }
    else
        ROS_INFO("TOO FEW MATCHES!");//*/
    /*/ get those rectified matched image points from ref ORB and cur all levels
    vector<cv::Point2f> refframe;
    vector<cv::Point2f> curframe;
    ros::Time homotimeb = ros::Time::now();
    if (mLandingPad->GetMatches_ful().size() > 8){
        cv::KeyPoint obj, cur;
        for( int j = 0; j < mLandingPad->GetMatches_ful().size(); j++ )
        {
            //-- Get the keypoints from the good matches
            // ref image should have been undistorted already
            obj = mLandingPad->GetallRefpoints()[mLandingPad->GetMatches_ful()[j].queryIdx];
            // project the current frame points to undistorted image frame
            cur = mLandingPad->GetallCurpoints()[mLandingPad->GetMatches_ful()[j].trainIdx];
            // project matches in different level to the first level, so we get one consistant match.
            switch(cur.octave) {
            case 0: break;
            default: {
                cur.pt.x = cur.pt.x * (1 << cur.octave);
                cur.pt.y = cur.pt.y * (1 << cur.octave);
                break;
            }
            }
            TooN::Vector<2> undistim;
            undistim[0] = cur.pt.x;
            undistim[1] = cur.pt.y;
            undistim = mCamera->UnProject(undistim);
            undistim = mCamera->Project_ud(undistim);
            cur.pt.x = undistim[0];
            cur.pt.y = undistim[1];
            curframe.push_back( cur.pt );
            // ref features are just on its first level
            refframe.push_back(obj.pt);
        }
    }
    else
        ROS_INFO("TOO FEW MATCHES!");//*/


    // homography of the object should be recovered here from those matches.
    ros::Time homonlyb = ros::Time::now();
    cv::Mat H(3,3, CV_64FC1);
    static gvar3<int> gvnMinRANSACinliers("MapMaker.MinRANSACinliers", 6, SILENT);
    static gvar3<int> gvnMaxRANSACiteration("MapMaker.MaxRANSACiteration", 500, SILENT);
    static gvar3<double> gvnMaxRANSACreprojectErr("MapMaker.MaxRANSACreprojectErr", 3, SILENT);
    int RANSACinliers = 0;

    if (curframe.size() > *gvnMinRANSACinliers)
        H = homographyRANSACsolver::findHomographyRANSAC(RANSACinliers, cv::Mat(refframe), cv::Mat(curframe), *gvnMinRANSACinliers, *gvnMaxRANSACiteration, *gvnMaxRANSACreprojectErr);
    //        H = cv::findHomography( cv::Mat(refframe), cv::Mat(curframe), CV_RANSAC, 3 );
    else
        ROS_INFO("TOO FEW MATCHES FOR HOMOGRAPHY COMPUTATION!");
    ros::Duration homonlytime = ros::Time::now() - homonlyb;

    // find a good way to check whether we have really obtain a correct homography estimate from this H.
    // e.g. number of keypoints in the polygon formed by the 4 corners of the landing pad.
    // and the polygon should be a Convex polygon!
    // if so label object been detected already,
    // and make the object subimage as the new template for ESM tracking.
    //    isObject_detected = false;// must comment this line when not debugging
    vector<cv::Point> scene_corners(4);
    if (H.at<double>(2, 2)){
        vector<cv::Point2f> obj_corners(4);// using keypoints on the 0 level of curnt img
        obj_corners[0] = cv::Point(0,0);
        obj_corners[1] = cv::Point( mLandingPad->GetRefimgs(0).cols, 0 );
        obj_corners[2] = cv::Point( mLandingPad->GetRefimgs(0).cols, mLandingPad->GetRefimgs(0).rows);
        obj_corners[3] = cv::Point( 0, mLandingPad->GetRefimgs(0).rows );
        cv::Mat scene_corners_;

        cv::perspectiveTransform( cv::Mat(obj_corners), scene_corners_, H);

        // those four corners have already been transformed to the undistorted image frame,
        // if we want to demonstrate it, we should project them to the distorted image / its original image.
        scene_corners_.copyTo(scene_corners);
        // if the polygon formed by the 4 corners is a convex polygon,
        // and then if the number of features detected in it is larger than a threshold (this is TOO tricky),
        // which is decided by a predifined threshold and the RANSAC inliers, assume the landing pad found

        // 1. check whether it is a convex polygon,
        PolygonClass checkpolygon;
        bool convexpolygon = false;
        checkpolygon = Polygon::ClassifyPolygon(scene_corners, 4);
        if ((checkpolygon == ConvexCCW) || (checkpolygon == ConvexCW)) convexpolygon = true;

        // 2. if so, check geometric relation of those 4 points and their gravity center
        cv::Point center_ = Polygon::mass_center(scene_corners);
        if ( convexpolygon && !Polygon::masscenter_distribute_normal(center_, scene_corners))
            convexpolygon = false;
        /*/ 3. weak constrain, check keypoints IN GOODMATCHES in these 4 corners of the current frame level [0]
        if (convexpolygon){
            int matchedpoints_in = 0;

            for (int leveli = 0; leveli < objLEVEL; leveli ++){
                vector<cv::KeyPoint> points = mLandingPad->GetCurpoints(0);// using keypoints on the 0 level of curnt img
                if (leveli < 1) continue;
//                for (int i = 0; i < mLandingPad->GetCurpoints(0).size(); i ++){
                for (int i = 0; i < mLandingPad->GetMatches(leveli).size(); i ++){
//                    cv::Point point = points[i].pt;
                    cv::Point point = points[mLandingPad->GetMatches(leveli)[i].queryIdx].pt;
                    if (Polygon::wn_PnPoly(point, scene_corners, 4) != 0)
                        matchedpoints_in ++;
                }
            }

//            if (matchedpoints_in > (mLandingPad->GetMatches(1).size()+mLandingPad->GetMatches(2).size())/2||
            if (matchedpoints_in > RANSACinliers)// homography confirmed!
            {
                isObject_detected = true;
                mObject_detected = k;
                // here we get H^0_1, the homography of the original reference subimage to the first image frame in which the landing pad was detected
                mPadHomography = H;
                mpadtrackerhomography = cvdopencv_helper::mat2toonmatrix<3, 3, double>(H);
            }
            ROS_INFO("MATCHED Points in polygon: %d, %d", matchedpoints_in, isObject_detected);
        }//*/
        // check all matches on all levels
        if (convexpolygon){
            int matchedpoints_in = 0;
            vector<cv::KeyPoint> points = mLandingPad->GetallCurpoints();// using keypoints on the 0 level of curnt img
            for (int i = 0; i < mLandingPad->GetMatches_ful().size(); i ++){
                cv::Point point = points[mLandingPad->GetMatches_ful()[i].queryIdx].pt;
                if (Polygon::wn_PnPoly(point, scene_corners, 4) != 0)
                    matchedpoints_in ++;
            }

            if (matchedpoints_in > mLandingPad->GetMatches_ful().size()/2.0 ||
                    matchedpoints_in > RANSACinliers)// homography confirmed!
            {
                isObject_detected = true;
                mObject_detected = k;
                // here we get H^0_1, the homography of the original reference subimage to the first image frame in which the landing pad was detected
                mPadHomography = H;
                mpadtrackerhomography = cvdopencv_helper::mat2toonmatrix<3, 3, double>(H);
            }
            ROS_INFO("MATCHED Points in polygon: %d, %d", matchedpoints_in, isObject_detected);
        }//*/
    }
    else
        ROS_INFO("H(2, 2): %f", H.at<double>(2, 2));

    ros::Duration homotime = ros::Time::now() - homotimeb;
    ROS_INFO("HOMOGRAPHY COMP time: %f, %f", homotime.toSec(), homonlytime.toSec());

    // calculation of the landing pad pose if detected
    if (isObject_detected){//(H.at<double>(2, 2)){//
        nPositive ++;
        // now search for those map points which could be reprojected to the image area of the landing pad
        // and store them to this keyframe.
        // only those map points visibal by the current frame are considered
        int mappinpad = 0;
        static gvar3<int> gvnShrinkPolygonfactor("MapMaker.ShrinkPolygonfactor", 0.8, SILENT);
        for (unsigned int i = 0; i < mMap.vpPoints.size(); i++) {
            MapPoint &p= *( mMap.vpPoints[i]);

            p.TData.Project(p.v3WorldPos, k.se3CfromW, mCamera.get());
            if (!p.TData.bInImage) {
                continue;
            }

            TooN::Vector<2> undistim;
            undistim = p.TData.v2Image;
            undistim = mCamera->UnProject(undistim);
            undistim = mCamera->Project_ud(undistim);
            cv::Point point(undistim[0], undistim[1]);

            // resize the polygon (make it smaller), so that to make sure only those map points belong to the pad should be included.
            //       but still can not solve a landing pad on a table, which should be done in RANSAC
            //            vector<cv::Point> shrink_corners(4);
            //            shrink_corners = scene_corners;
            //            shrink_corners = Polygon::shrinkpolygon(shrink_corners, *gvnShrinkPolygonfactor);
            int num = Polygon::cn_PnPoly(point, scene_corners, 4);
            if ( num != 0){
                k.mMapPointsInPad.push_back(mMap.vpPoints[i]);
                mappinpad ++;
            }
        }
        ROS_INFO("INSIDE mappoints: %d", mappinpad);

        static int numlogpad = 1;
        // for debug
        if (false){
            pos_log_ <<numlogpad<< " scene_corners: (" << scene_corners[0].x << ", " << scene_corners[0].y << ") ("
                    << scene_corners[1].x << ", " << scene_corners[1].y << ") ("
                    << scene_corners[2].x << ", " << scene_corners[2].y << ") ("
                    << scene_corners[3].x << ", " << scene_corners[3].y << ") "
                    << k.mMapPointsInPad.size()<<" " << ros::Time::now().toSec() << "\n";
            // draw map points
            cv::Mat inlinepoints = mLandingPad->GetCurimgs(0);
            cv::drawKeypoints(inlinepoints, mLandingPad->GetCurpoints(0), inlinepoints, cvScalar(255, 255, 0));
            for (unsigned int i = 0; i < k.mMapPointsInPad.size(); i++) {
                MapPoint &p= *(k.mMapPointsInPad[i]);
                p.TData.Project(p.v3WorldPos, k.se3CfromW, mCamera.get());
                if (!p.TData.bInImage) {
                    continue;
                }
                TooN::Vector<2> undistim;
                undistim = p.TData.v2Image;
                undistim = mCamera->UnProject(undistim);
                undistim = mCamera->Project_ud(undistim);

                int x = undistim[0];
                int y = undistim[1];
                cv::Point point(undistim[0], undistim[1]);

                pos_log_ << point.x << " " << point.y<< " ";

                int num = Polygon::cn_PnPoly(point, scene_corners, 4);
                if ( num != 0){
                    rectangle(inlinepoints, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                              (cv::Scalar) cv::Vec3b(0, 0, 255), CV_FILLED);
                }
                else
                    rectangle(inlinepoints, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                              (cv::Scalar) cv::Vec3b(0, 255, 0), CV_FILLED);
            }
            pos_log_ << "\n";
            pos_log_ << ros::Time::now().toSec() << "\n";
            cv::line( inlinepoints, scene_corners[0], scene_corners[1], cvScalar(255, 0, 0), 4 );
            cv::line( inlinepoints, scene_corners[1], scene_corners[2], cvScalar( 0, 255, 0), 4 );
            cv::line( inlinepoints, scene_corners[2], scene_corners[3], cvScalar( 0, 0, 255), 4 );
            cv::line( inlinepoints, scene_corners[3], scene_corners[0], cvScalar( 255, 255, 0), 4 );
            cv::imshow("inlinepoints", inlinepoints);
            string num("inlinepoints");
            static int inlinenum = 0;
            stringstream temp;
            temp  << inlinenum;
            temp  << ".jpg";
            num += temp.str();
            //            cv::imwrite(num, inlinepoints);
            inlinenum ++;
            cv::waitKey(10);

            pos_log_ << ros::Time::now().toSec() << "\n";
        }
        // if the map points we have in all those frames so far(actually = in the current frame), in which the landing pad is detected, reach a threshold number
        // them landing pad plane could be initialised, using RANSAC ---> model(m-1) with inliers(m-1) > n(number of all mappoints)
        // for this, original PTAM ini method or RANSAC in PCL from Konstantin could be used.
        isLandingPoseGetCurrent = false;
        static gvar3<int> gvnMinMappointLandingpadRANSAC("MapMaker.MinMappointLandingpadRANSAC", 20, SILENT);
        if ( !isLandingpadPoseGet){
            if ( mappinpad > *gvnMinMappointLandingpadRANSAC &&
                 CalcLandingpadPlane(k.mMapPointsInPad, mSe3Landingpadfromworld, mLandingpadNormal, mLandingpadMean)){
                k.mSe3Landingpadfromworld = mSe3Landingpadfromworld;
                k.mLandingpadNormal = mLandingpadNormal;
                isLandingpadPoseGet = true;
                isLandingPoseGetCurrent = true;
                iniPadCameraPoseWorld = k.se3CfromW.inverse().get_translation();
            }
        }
        // them in those sucessive frames, a incremental method should be designed to optimise the pose of the landing pad,
        // not just using all those map points till now in each frame.
        // since only concensus is considered in RANSAC, number of inliers/outliers cannot be used here
        // but last bestnormal can be used to reduce iteration times -----> model(m), -----> better pad pose
        else{
            if (mappinpad > *gvnMinMappointLandingpadRANSAC &&
                    CheckInlier_Refine(k.mMapPointsInPad, mSe3Landingpadfromworld, mLandingpadNormal, mLandingpadMean)){
                k.mSe3Landingpadfromworld = mSe3Landingpadfromworld;
                k.mLandingpadNormal = mLandingpadNormal;
                isLandingPoseGetCurrent = true;
            }
        }
        // if a model of the plane model(m-1) is obtained, pose of the landing pad can be estimated
        // using the intersections of the four rays and this plane
        // sometimes could only use the translation of this plane center, assuming parallel to ground level
        if (isLandingPoseGetCurrent){
            std::vector<TooN::Vector<3> > Xwpadcorners(4);//
            TooN::Vector<3> Xwpadcenter = TooN::makeVector( 0, 0, 0 );
            TooN::SE3<> Twc = k.se3CfromW.inverse();
            double zpad = mLandingpadMean[2];// assuming parallel
            for (int i = 0; i < 4; i ++){
                TooN::Vector<2> nxpad = mCamera->UnProject_ud(TooN::makeVector( scene_corners[i].x, scene_corners[i].y ));
                TooN::Vector<3> xn = TooN::makeVector(nxpad[0], nxpad[1], 1.0);

                // TODO: generall case:  Xw = s*Rwc*Xn + twc; np*(Xw - Xp) = 0     ==>s and Xw

                // just assume detected pad plane is parallel to ground plane
                TooN::Vector<3> twc = Twc.get_translation();
                TooN::SO3<> rwc = Twc.get_rotation();
                double scalef = (zpad - twc[2])/(rwc.get_matrix()[2]*xn);

                Xwpadcorners[i][0] = scalef*(rwc.get_matrix()[0]*xn) + twc[0];
                Xwpadcorners[i][1] = scalef*(rwc.get_matrix()[1]*xn) + twc[1];
                Xwpadcorners[i][2] = zpad;

                Xwpadcenter += Xwpadcorners[i];
            }
            mPadCenterWorld = Xwpadcenter/4.0;
            mPadCornersWorld = Xwpadcorners;

            static TooN::Vector<3>  inipose = mPadCenterWorld;
            iniPadCenterWorld = inipose;

            if (pos_log_.is_open()){
                pos_log_ << numlogpad <<" " << ros::Time::now().toSec()<< " " << zpad
                         << " " << mPadCenterWorld[0] << " " << mPadCenterWorld[1]
                         << " " << mPadCenterWorld[2] << " " << mLandingpadNormal[0]
                         << " " << mLandingpadNormal[1] << " " << mLandingpadNormal[2]
                         << " " << Twc.get_translation()[0] << " " << Twc.get_translation()[1]
                         << " " << Twc.get_translation()[2] << " " << isFinishPadDetection  << "\n";
                numlogpad ++;
            }

            // if a model of the landing pad pose is obtained, save this keyframe with important landing pad information:
            // 4 corners of the landing pad image area; map points related to the landing pad
            k.islandingpadDetected = true;
            k.mPadCorners = scene_corners;
            k.mPadCenterWorld = mPadCenterWorld;
            k.mPadCornersWorld = mPadCornersWorld;
            boost::shared_ptr<KeyFrame> pK(new KeyFrame);
            *pK = k;
            mPaddetectedKeyframes.push_back(pK);

            // at some point, refining of the landing pad pose should be stopped,
            // and calculate the final pose, rejecting those normal vector which are
            // very different from the final best normal.
            // this should happen when camera is nearly above the pad, begin the landing phase
            //            double disxy = sqrt(Polygon::square(Twc.get_translation()[0]-mPadCenterWorld[0])+
            //                                Polygon::square(Twc.get_translation()[1]-mPadCenterWorld[1]));
            // use the inipose to control end, easier for way setpoints node.
            double disxy = sqrt(Polygon::square(Twc.get_translation()[0]-iniPadCenterWorld[0])+
                                Polygon::square(Twc.get_translation()[1]-iniPadCenterWorld[1]));
            static gvar3<double> gvnStopDetectionDisxy("MapMaker.StopDetectionDisxy", 0.4, SILENT);
            if (disxy < *gvnStopDetectionDisxy){
                // reject abnomal center, and then refine it with inliers
                TooN::Vector<3> mMeanPadCenter = TooN::makeVector(0, 0, 0);
                for (int i = 0; i < mPaddetectedKeyframes.size(); i ++){
                    mMeanPadCenter = mMeanPadCenter+mPaddetectedKeyframes[i]->mPadCenterWorld;
                }
                mMeanPadCenter = mMeanPadCenter / mPaddetectedKeyframes.size();

                double dis2mean = 0;
                for (int i = 0; i < mPaddetectedKeyframes.size(); i ++){
                    dis2mean = sqrt((mPaddetectedKeyframes[i]->mPadCenterWorld - mMeanPadCenter)*
                                    (mPaddetectedKeyframes[i]->mPadCenterWorld - mMeanPadCenter));
                    static gvar3<double> gvnInlierPadcenterDis("MapMaker.InlierPadcenterDis", 0.3, SILENT);
                    if (dis2mean > *gvnInlierPadcenterDis){
                        mPaddetectedKeyframes[i]->islandingpadDetected = false;
                    }
                }

                TooN::Vector<3> PadCenter = TooN::makeVector(0, 0, 0);
                int numinlierframe = 0;
                for (int i = 0; i < mPaddetectedKeyframes.size(); i ++){
                    if (mPaddetectedKeyframes[i]->islandingpadDetected){
                        PadCenter += mPaddetectedKeyframes[i]->mPadCenterWorld;
                        numinlierframe ++;
                    }
                }
                if (numinlierframe)
                    mMeanPadCenter = PadCenter/numinlierframe;

                mPadCenterWorld = mMeanPadCenter;

                static gvar3<int> gvnNeedStopDetection("MapMaker.NeedStopDetection", 1, SILENT);
                if (*gvnNeedStopDetection!=0)
                    isFinishPadDetection = true;

                finishPadCameraPoseWorld = k.se3CfromW.inverse().get_translation();

                numlogpad = 0;
                if (pos_log_.is_open()){
                    pos_log_ << numlogpad <<" " << ros::Time::now().toSec() << " " << numinlierframe
                             << " " << mPadCenterWorld[0] << " " << mPadCenterWorld[1]
                             << " " << mPadCenterWorld[2] << " " << mLandingpadNormal[0]
                             << " " << mLandingpadNormal[1] << " " << mLandingpadNormal[2]
                             << " " << Twc.get_translation()[0] << " " << Twc.get_translation()[1]
                             << " " << Twc.get_translation()[2] << " " << isFinishPadDetection << "\n";
                    numlogpad ++;
                }
            }
        }
    }
    else
        nNegative ++;

    if (isLandingPoseGetCurrent)
        ROS_INFO("LANDING PAD POSE GOT: %f, %f, %f", mPadCenterWorld[0], mPadCenterWorld[1], mPadCenterWorld[2]);

    /////////////////////////////////////////////////////////////////////////////////
    /*/ plot 4 corners of the landing pad for debug
    static int imsavenum = 0;
    vector<cv::Point> draw_corners(4);
    if (isObject_detected){ // isObject_detected && !mLandingPad->pad_detected
//    if (H.at<double>(2, 2)){
        for (int i = 0; i < objLEVEL; i ++){
            // show match for debug
            cv::Mat img_matches;
            cv::drawMatches(mLandingPad->GetRefimgs(i), mLandingPad->GetRefpoints(i),
                            mLandingPad->GetCurimgs(0), mLandingPad->GetCurpoints(0),
                            mLandingPad->GetMatches(i), img_matches, cv::Scalar( 0, 255, 0), cv::Scalar( 255, 255, 0));

            if (true){ // homography got
                // then we need to project those undistorted image points to the original image frame
                if (i == 0) {
                    for (int num_corner = 0; num_corner < 4; num_corner ++){
                        draw_corners[num_corner] = scene_corners[num_corner];
                        TooN::Vector<2> distim;
                        distim[0] = draw_corners[num_corner].x;
                        distim[1] = draw_corners[num_corner].y;
                        distim = mCamera->UnProject_ud(distim);
                        distim = mCamera->Project(distim);
                        draw_corners[num_corner].x = distim[0];
                        draw_corners[num_corner].y = distim[1];
                    }
                }
                //-- Draw lines between the corners (the mapped object in the scene )
                cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 4 );
                cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 4 );
                cv::line( img_matches, draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 0, 255), 4 );
                cv::line( img_matches, draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 255, 255, 0), 4 );
//                cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 4 );
//                cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 4 );

                cv::rectangle( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar(255, 0, 0), 4 );
                cv::rectangle( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 0, 255, 0), 4 );
                cv::rectangle( img_matches, draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 0, 0, 255), 4 );
                cv::rectangle( img_matches, draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 255, 255, 0), 4 );
                cv::Point center = Polygon::mass_center(draw_corners);
                cv::circle(img_matches, center + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), 3, cvScalar( 0, 255, 0), 2);

                if (i < 1) continue;

                string num("pad");
                stringstream temp;
                temp  << imsavenum;
                temp  <<  "_";
                temp  << i;
                temp  << ".jpg";
                num += temp.str();
                cv::imwrite(num, img_matches);
            }

            // save rectangle for paper figure
            if (false){ // homography got
                cv::Mat img_matches(k.aLevels[i].im.size().x, k.aLevels[i].im.size().x, CV_8UC3, mLandingPad->GetCurimgs(0).data);
                cv::drawKeypoints(img_matches, mLandingPad->GetCurpoints(i), img_matches, cvScalar(255, 0, 0));
                // then we need to project those undistorted image points to the original image frame
                if (i == 0) {
                    for (int num_corner = 0; num_corner < 4; num_corner ++){
                        draw_corners[num_corner] = scene_corners[num_corner];
                        TooN::Vector<2> distim;
                        distim[0] = draw_corners[num_corner].x;
                        distim[1] = draw_corners[num_corner].y;
                        distim = mCamera->UnProject_ud(distim);
                        distim = mCamera->Project(distim);
                        draw_corners[num_corner].x = distim[0];
                        draw_corners[num_corner].y = distim[1];
                    }
                }
                //-- Draw lines between the corners (the mapped object in the scene )
                cv::line( img_matches, draw_corners[0], draw_corners[1], cvScalar(255, 0, 0), 4 );
                cv::line( img_matches, draw_corners[1], draw_corners[2], cvScalar( 0, 255, 0), 4 );
                cv::line( img_matches, draw_corners[2], draw_corners[3], cvScalar( 0, 0, 255), 4 );
                cv::line( img_matches, draw_corners[3], draw_corners[0], cvScalar( 255, 255, 0), 4 );
//                cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 4 );
//                cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 4 );

                cv::rectangle( img_matches, draw_corners[0]-cv::Point(2,2), draw_corners[0] +cv::Point(2,2), cvScalar(255, 0, 0), 4 );
                cv::rectangle( img_matches, draw_corners[1]-cv::Point(2,2), draw_corners[1] +cv::Point(2,2), cvScalar( 0, 255, 0), 4 );
                cv::rectangle( img_matches, draw_corners[2]-cv::Point(2,2), draw_corners[2] +cv::Point(2,2), cvScalar( 0, 0, 255), 4 );
                cv::rectangle( img_matches, draw_corners[3]-cv::Point(2,2), draw_corners[3] +cv::Point(2,2), cvScalar( 255, 255, 0), 4 );
                cv::Point center = Polygon::mass_center(draw_corners);
                cv::circle(img_matches, center, 3, cvScalar( 0, 255, 0), 2);

                if (i < 1) continue;

                string num("padrect");
                stringstream temp;
                temp  << imsavenum;
                temp  <<  "_";
                temp  << i;
                temp  << ".jpg";
                num += temp.str();
                cv::imwrite(num, img_matches);
            }

//            switch(i) {
//            case 0: cv::imshow("match0", img_matches); break;
//            case 1: cv::imshow("match1", img_matches); break;
//            case 2: cv::imshow("match2", img_matches); break;
////            case 3: cv::imshow("match3", img_matches); break;
//            default: break;
//            }
            cv::waitKey(10);
        }
        imsavenum ++;
    }
//////////////////////////////////////////////*/
    // plot 4 corners of the landing pad for debug
    static gvar3<int> gvnisSaveMatches("MapMaker.isSaveMatches", 0, SILENT);
    static int imsavenum = 0;
    vector<cv::Point> draw_corners(4);
    if ((*gvnisSaveMatches!=0) && isObject_detected){ //H.at<double>(2, 2)// isObject_detected && !mLandingPad->pad_detected
        for (int i = 1; i < objLEVEL; i ++){
            for (int j = 0; j <2; j ++)
                // show match for debug
            {
                // TODO: draw true and false matches in different color
                if (true){ // homography got
                    std::vector<cv::KeyPoint> curpoints = mLandingPad->GetCurpoints(j);
                    cv::Mat curimg = mLandingPad->GetCurimgs(j);
                    cv::drawKeypoints(curimg, curpoints, curimg, cvScalar(255, 255, 0));
                    cv::Mat img_matches;
                    cv::drawMatches(mLandingPad->GetRefimgs(i), mLandingPad->GetallRefpoints(),
                                    curimg, mLandingPad->GetallCurpoints(),
                                    mLandingPad->GetMatches((i-1)*2+j), img_matches,
                                    cv::Scalar( 0, 255, 0), cv::Scalar( 255, 255, 0),
                                    vector<char>(),  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                    // then we need to project those undistorted image points to the original image frame
                    if (true) {
                        for (int num_corner = 0; num_corner < 4; num_corner ++){
                            draw_corners[num_corner] = scene_corners[num_corner];
                            TooN::Vector<2> distim;
                            distim[0] = draw_corners[num_corner].x;
                            distim[1] = draw_corners[num_corner].y;
                            distim = mCamera->UnProject_ud(distim);
                            distim = mCamera->Project(distim);
                            draw_corners[num_corner].x = distim[0];
                            draw_corners[num_corner].y = distim[1];
                            draw_corners[num_corner].x /= 1 << j;
                            draw_corners[num_corner].y /= 1 << j;
                        }
                    }
                    //-- Draw lines between the corners (the mapped object in the scene )
                    cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 8 );
                    cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 8 );
                    cv::line( img_matches, draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 0, 255), 8 );
                    cv::line( img_matches, draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 255, 255, 0), 8 );
                    //                cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 4 );
                    //                cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 4 );

                    //                    cv::rectangle( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar(255, 0, 0), 4 );
                    //                    cv::rectangle( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 0, 255, 0), 4 );
                    //                    cv::rectangle( img_matches, draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 0, 0, 255), 4 );
                    //                    cv::rectangle( img_matches, draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 255, 255, 0), 4 );
                    cv::Point center = Polygon::mass_center(draw_corners);
                    cv::circle(img_matches, center + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), 3, cvScalar( 0, 255, 0), 2);

                    if (i < 1) continue;

                    string num("pad");
                    stringstream temp;
                    temp  << imsavenum;
                    temp  <<  "_";
                    temp  << i;
                    temp  << j;
                    temp  << ".jpg";
                    num += temp.str();
                    cv::imwrite(num, img_matches);
                }
                //                switch(i) {
                //                case 0: cv::imshow("match0", img_matches); break;
                //                case 1:
                //                    if (j == 0)
                //                        cv::imshow("match0", img_matches);
                //                    else if (j ==1 )
                //                        cv::imshow("match1", img_matches);
                //                    break;
                //                case 2:
                //                    if (j == 0)
                //                        cv::imshow("match2", img_matches);
                //                    else if (j ==1 )
                //                        cv::imshow("match3", img_matches);
                //                    break;
                //    //            case 3: cv::imshow("match3", img_matches); break;
                //                default: break;
                //                }
                //                cv::waitKey(10);
            }
        }
        imsavenum ++;
    }//*/

    /*/ using ref ORB plot 4 corners of the landing pad for debug
    static gvar3<int> gvnisSaveMatches("MapMaker.isSaveMatches", 0, SILENT);
    static gvar3<int> gvnfirstleve("MapMaker.RefORBfirstLevel", 1, SILENT);
    static int imsavenum = 0;
    vector<cv::Point> draw_corners(4);
    if ((*gvnisSaveMatches!=0) ){ //&& isObject_detected// isObject_detected && !mLandingPad->pad_detected
        int i = *gvnfirstleve;
        for (int j = 0; j <2; j ++)
        // show match for debug
        {
            // TODO: draw true and false matches in different color
            std::vector<cv::KeyPoint> curpoints = mLandingPad->GetCurpoints(j);
            cv::Mat curimg = mLandingPad->GetCurimgs(j);
            cv::drawKeypoints(curimg, curpoints, curimg, cvScalar(255, 255, 0));
            cv::Mat img_matches;
            cv::drawMatches(mLandingPad->GetRefimgs(i), mLandingPad->GetallRefpoints(),
                            curimg, mLandingPad->GetallCurpoints(),
                            mLandingPad->GetMatches(j), img_matches,
                            cv::Scalar( 0, 255, 0), cv::Scalar( 255, 255, 0),
                            vector<char>(),  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            if (true){ // homography got
                // then we need to project those undistorted image points to the original image frame
                if (true) {
                    for (int num_corner = 0; num_corner < 4; num_corner ++){
                        draw_corners[num_corner] = scene_corners[num_corner];
                        TooN::Vector<2> distim;
                        distim[0] = draw_corners[num_corner].x;
                        distim[1] = draw_corners[num_corner].y;
                        distim = mCamera->UnProject_ud(distim);
                        distim = mCamera->Project(distim);
                        draw_corners[num_corner].x = distim[0];
                        draw_corners[num_corner].y = distim[1];
                        draw_corners[num_corner].x /= 1 << j;
                        draw_corners[num_corner].y /= 1 << j;
                    }
                }
                //-- Draw lines between the corners (the mapped object in the scene )
                cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 4 );
                cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 4 );
                cv::line( img_matches, draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 0, 255), 4 );
                cv::line( img_matches, draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 255, 255, 0), 4 );
                //                cv::line( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar(255, 0, 0), 4 );
                //                cv::line( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), cvScalar( 0, 255, 0), 4 );

                cv::rectangle( img_matches, draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[0] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar(255, 0, 0), 4 );
                cv::rectangle( img_matches, draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[1] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 0, 255, 0), 4 );
                cv::rectangle( img_matches, draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[2] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 0, 0, 255), 4 );
                cv::rectangle( img_matches, draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)-cv::Point(2,2), draw_corners[3] + cv::Point( mLandingPad->GetRefimgs(i).cols, 0)+cv::Point(2,2), cvScalar( 255, 255, 0), 4 );
                cv::Point center = Polygon::mass_center(draw_corners);
                cv::circle(img_matches, center + cv::Point( mLandingPad->GetRefimgs(i).cols, 0), 3, cvScalar( 0, 255, 0), 2);

                string num("pad");
                stringstream temp;
                temp  << imsavenum;
                temp  <<  "_";
                temp  << i;
                temp  << j;
                temp  << ".jpg";
                num += temp.str();
//                cv::imwrite(num, img_matches);
            }
            switch(j) {
            case 0: cv::imshow("match0", img_matches); break;
            case 1: cv::imshow("match1", img_matches); break;
//            case 2: cv::imshow("match2", img_matches); break;
////            case 3: cv::imshow("match3", img_matches); break;
            default: break;
            }
        }


        cv::waitKey(10);

        imsavenum ++;
    }//*/

    // get ready for later frame tracking.
    if (false){//isObject_detected
        // find the current sub-image for later ESM tracking.
        if (!mLandingPad->pad_detected){// this also labels the first frame when pad is detected
            // test: chose the original reference template from the correct level, e.g. using height information,
            // as the reference template for tracking, but this should near be used, not qualified to the ESM
            //          mLandingPad->makepadtrakerthisfromorigin(2);// here let's just use level 1
            //          mLandingPad->makepadtrackerthis(k.aLevels[3].im);// debug, try ESM with whole frames
            //          mLandingPad->makepadtrackerlast(k.aLevels[3].im);//

            // used: copy the detected image area as the reference template for later tracking
            // first, simply, decide the rectanglar subimage of the detected pad. Also ini lastframe
            // this subimage should be come from the undistorted/rectified image.
            cv::Point lefttop, rightbottom;
            int level = 1;// on which level should we chose the subimage?
            int i = 1 << level;
            if (Polygon::findmaxsubimg(scene_corners, cv::Size(k.aLevels[0].im.size().x, k.aLevels[0].im.size().y), lefttop, rightbottom))
            {
                ImageRef lefttop_(lefttop.x/i, lefttop.y/i);
                ImageRef padsize((rightbottom.x-lefttop.x)/i, (rightbottom.y-lefttop.y)/i);
                // TODO: here should use specified function to get the subimage in the rectified image, without rectifing the whole image

                mPadsubimg.copy_from( k.aLevels[level].im.sub_image(lefttop_, padsize) );

                msubimglt = lefttop_; msubimgsize = padsize;
                mLandingPad->makepadtrackerthis(mPadsubimg);
                mLandingPad->makepadtrackerlast(mPadsubimg);

                // also initialise H^1_2 with the location of the subimage in this frame
                mTsubimagelast = TooN::Ones;
                mTsubimagelast(2, 0) = 0;
                mTsubimagelast(2, 1) = 0;
                mTsubimagelast(0, 2) = lefttop_.x;
                mTsubimagelast(1, 2) = lefttop_.y;

                mLandingPad->pad_detected = true;
            }
            else
                isObject_detected = false;
        }
    }
}

void MapMaker::TracklandingpadESM(KeyFrame &k)
{
    // this means we have come to the later frames, do tracking
    // the current subimage for tracking, i.e. calculating the homography, should be
    // located using the motion model from ptam/tracker_.
    // also in ESM only consider those image points inside the predicted polygon
    // corresponding to the reference template

    ros::Time time_esm = ros::Time::now();
    // at first, just consider the same subimage as detected/tracked in the last frame.
    // used: or just try the whole current image!! And the same homography of last frame
    // be used as ini location of ESM.
    // then, we should use the subimage estimated by the motion model from ptam/tracker_
    //    Image<byte> padsubimg;
    //    padsubimg.copy_from( k.aLevels[0].im.sub_image(msubimglt, msubimgsize) );
    //    mLandingPad->copypadtracker_this2last(padsubimg);
    mLandingPad->makepadtrackerthis(k.aLevels[1].im);// debug, level should be the same as last frame, try ESM with whole frames

    std::cout << "fine2" << std::endl;

    // main ESM estimator
    mTsubimagethis = mLandingPad->padtrackermain(mTsubimagelast);// actually, last subimage to this frame

    // then this is what we need, landing pad in last image to landing pad in this image
    // i.e. H^{n-1}_n = H^sub_thisframe * (H^sub_lastframe).inv
    cv::Mat mTsubimgmat = cvdopencv_helper::toonmatrix2mat<3, 3>(mTsubimagelast);
    mHlast2thisframe = mTsubimagethis * cvdopencv_helper::mat2toonmatrix<3, 3, double> (mTsubimgmat.inv());

    // again, copy the subimage corresponding to landing pad to make a template for later padtracking.

    ros::Duration duration_esm = ros::Time::now() - time_esm;
    ROS_INFO("ESM COMP time: %f", duration_esm.toSec());

    std::cout << "fine3" << std::endl;

    static int debugnum = 0;// debug, save the ESM result related images
    if (debugnum == 0){
        // transform the current image, and save it, to compare with the reference template, for debug
        Image<byte> transimg;
        transimg.resize(k.aLevels[1].im.size());

        CVD::img_save(mObject_detected.aLevels[0].im, "lastframe1.jpg");
        CVD::img_save(mPadsubimg, "lastsubref1.jpg");
        CVD::img_save(k.aLevels[0].im, "this1.jpg");
        //        CVD::Internal::transform_perspective(transimg, mObject_detected.aLevels[0].im, mHlast2thisframe);
        CVD::Internal::transform_perspective(transimg, mPadsubimg, mTsubimagethis);
        std::cout << "fine4" << std::endl;
        CVD::img_save(transimg, "translast1.jpg");

        debugnum = 1;
    }
}

bool MapMaker::AddReflandingpadFrame(KeyFrame &k)
{
    mReflandingpadFrame = k;

    // also compute descriptor of the ref images in different level.
    // since there's drametic distortion for wide angle view lens, the ref img should have been undistorted
    // before feature detection, this will benefit the homography calculation, which assume a perspective
    // camera model, without the considatration of distortion.
    static gvar3<int> gvnRefORBfirstLevel("MapMaker.RefORBfirstLevel", 1, SILENT);
    for (int i = 0; i < objLEVEL; i ++){
        // retrieve opencv img
        cv::Mat refimg(k.aLevels[i].im.size().y, k.aLevels[i].im.size().x, CV_8UC1, k.aLevels[i].im.data());
        cv::GaussianBlur(refimg, refimg, cv::Size(5, 5), 0.75/(i+1), 0.75/(i + 1));

        // retrieve opencv keypoints
        vector<cv::KeyPoint> points;
        cv::KeyPoint point;
        for (unsigned int j = 0; j < k.aLevels[i].vMaxCorners.size(); j ++){
            point.pt.x = k.aLevels[i].vMaxCorners[j].x;
            point.pt.y = k.aLevels[i].vMaxCorners[j].y;
            point.size = 15;
            point.octave = i;
            points.push_back(point);
        }
        //        mLandingPad->padfeature_detect(refimg, points);

        // compute opencv descriptors
        cv::Mat descriptors;
        mLandingPad->padfeature_descript(refimg, points, descriptors);// those points cannot be descript will be remove, in opencv function.

        mLandingPad->load_refcorners(points, i);
        mLandingPad->load_refdescriptors(descriptors, i);

        if (i == *gvnRefORBfirstLevel)
            mLandingPad->ini_allrefcorners(points);
        if (i > *gvnRefORBfirstLevel){
            for (int j = 0; j < points.size(); j ++)
                mLandingPad->add_allrefcorners(points[j]);
        }

        ROS_INFO("FEATURE AND DESCRIPTOR NUMS: %d %d", points.size(), descriptors.rows);

        // pass ref img to landing object
        mLandingPad->load_refimgs(refimg, i);

        // Also add image template of different levels for later tracking.
        // Later tracking should use multi-resolution method/ multi-scale method, heuristic method,
        // e.g. using height information to chose the right level template.
        mLandingPad->makepadtrackerOrigin(k.aLevels[i].im, i);
    }
    mLandingPad->load_allrefdescriptors(*gvnRefORBfirstLevel);//*/

    /*/ use ORB for ref image on the 0/1 level
    static gvar3<int> gvnRefORBfirstLevel("MapMaker.RefORBfirstLevel", 1, SILENT);
    int i = *gvnRefORBfirstLevel;// chose first the ref image level
    cv::Mat refimg(k.aLevels[i].im.size().y, k.aLevels[i].im.size().x, CV_8UC1, k.aLevels[i].im.data());
    cv::GaussianBlur(refimg, refimg, cv::Size(5, 5), 0.75/(i+1), 0.75/(i + 1));
    // retrieve opencv keypoints and descriptors
    vector<cv::KeyPoint> points;
    cv::Mat descriptors;
    mLandingPad->padfeature_detect(refimg, points);
    mLandingPad->padfeature_descript(refimg, points, descriptors);
    mLandingPad->add_allORBrefcorners(points);
    mLandingPad->load_allORBrefdescriptors(descriptors);
    mLandingPad->load_refimgs(refimg, i);//*/

    std::cout<< "ref detector and descriptor: " << mLandingPad->GetallRefpoints().size()
             << ", " << mLandingPad->GetallRefDescriptors().rows << std::endl;
    return true;
}

// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so 
// just dumps it on the top of the mapmaker's queue to 
// be dealt with later, and return.
bool MapMaker::AddKeyFrame(KeyFrame &k)
{
    boost::mutex::scoped_lock lock(MappingEnabledMut);
    if (mbMappingEnabled) {
        boost::shared_ptr<KeyFrame> pK(new KeyFrame);
        *pK = k;
        if (!k.nSourceCamera)
        {
            mvpKeyFrameQueue.push_back(pK);
        }
        else
        {
            mvpKeyFrameQueueSec[0].push_back(pK);
        }
        return true;
    } else {
        return false;
    }
//    lock.unlock();
}
bool MapMaker::AddKeyFrameDual(KeyFrame &k, KeyFrame &ksec)
{
    boost::mutex::scoped_lock lock(MappingEnabledMut);
    if (mbMappingEnabled) {
        if (k.nSourceCamera || !ksec.nSourceCamera)
        {
            cerr << "Keyframes identity error!!!" << endl;
            return false;
        }

        boost::shared_ptr<KeyFrame> pK(new KeyFrame);
        boost::shared_ptr<KeyFrame> pKsec(new KeyFrame);
        *pK = k;
        *pKsec = ksec;
        pK->mAssociateKeyframe = true;
        pKsec->mAssociateKeyframe = true;
        mvpKeyFrameQueue.push_back(pK);
        mvpKeyFrameQueueSec[0].push_back(pKsec);
        // label its associated kf from the master camera with associated.
        // which is the lated kf added to the mvpKeyFrameQueue
        mvpKeyFrameQueue[mvpKeyFrameQueue.size()-1]->nAssociatedKf = mvpKeyFrameQueueSec[0].size()-1;
        mvpKeyFrameQueueSec[0][mvpKeyFrameQueueSec[0].size()-1]->nAssociatedKf = mvpKeyFrameQueue.size()-1;
        return true;
    } else {
        return false;
    }
//    lock.unlock();
}

bool MapMaker::AddKeyFrameSec(KeyFrame &ksec)
{
    assert(ksec.nSourceCamera > 0);

    boost::mutex::scoped_lock lock(MappingEnabledMut);
    if (!mbMappingEnabled)
        return false;

    boost::shared_ptr<KeyFrame> pKsec(new KeyFrame);
    *pKsec = ksec;
    int adcamIndex = ksec.nSourceCamera - 1;
    pKsec->mAssociateKeyframe = true;
    mvpKeyFrameQueueSec[adcamIndex].push_back(pKsec);
    // label its associated kf from the master camera with associated.
    // which is the lated kf added to the mvpKeyFrameQueue
    mvpKeyFrameQueueSec[adcamIndex][mvpKeyFrameQueueSec[adcamIndex].size()-1]->nAssociatedKf = mvpKeyFrameQueue.size()-1;

    return true;
}

// Mapmaker's code to handle incoming key-frames.
// for dual camera case, kfs from two cameras are added separately,
// but those keyframes from the two cameras should be paired, except the initialised one
void MapMaker::AddKeyFrameFromTopOfQueue()
{
    cout << "Adding " << mvpKeyFrameQueue.size()<< " Keyframes to the local map..." << endl;
    boost::mutex::scoped_lock lockqueue(MappingEnabledMut);
    if(mvpKeyFrameQueue.size() == 0)
        return;

    static gvar3<int> gvnFixedFrameSize("MapMaker.FixedFrameSize", 3, SILENT);
    static gvar3<int> gvnVOonly("MapMaker.VOonly", 0, SILENT);

//    for (int i = 0; i < mMap.vpKeyFrames.size(); i ++){
//        if (mMap.vpKeyFrames[i]->mAssociateKeyframe)
//            cout << i <<": " << mMap.vpKeyFrames[i]->nAssociatedKf << ", " << mMap.vpKeyFrames[i]->mAssociatedinFinalQueue <<endl;
//    }
    // yang, original PTAM problem: when mapping thread is busy, it may not be able to add
    // a keyframe in time. Then tracking thread may add too many very close keyframes to the
    // waiting list, if the quadrotor flys fast. So we need to check those kf in the waiting
    // list again.
    bool neednewkf = true;
    bool neednewkfsec = true;
    bool usingDualimg = true;
    boost::shared_ptr<KeyFrame> pK, pK2[AddCamNumber];

    // force directly add kfs from the two cams together if there's association! <= check based on both kfs
    // then the association number would match each kf's own number
    /// TODO: allow individual additional kfs to be added, as I tried and abondoned before. This can improve the robustness of the system
    int nKfmin = mvpKeyFrameQueue.size();
    for (int i = 0; i < AddCamNumber; i ++){
        if (mvpKeyFrameQueueSec[i].size()== 0)
            usingDualimg = false;
        else if (mvpKeyFrameQueueSec[i].size() < nKfmin)
            nKfmin = mvpKeyFrameQueueSec[i].size();
    }

    for (int i = 0; i < nKfmin; i ++){
        pK = mvpKeyFrameQueue[0];
        mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
        cout << "current kf and existing kfs: " << pK->id << ", " << mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->id << endl;
        if (i != 0) /// in this case, not likely to add it
            neednewkf = NeedNewKeyFrame(pK);
        cout << "need new kf from first cam?: " << neednewkf << endl;

        /// Add kfs whenever one of the kfs should be added
        if (usingDualimg){
            for (int cn = 0; cn < AddCamNumber; cn ++){
                pK2[cn] = mvpKeyFrameQueueSec[cn][0];
                mvpKeyFrameQueueSec[cn].erase(mvpKeyFrameQueueSec[cn].begin());
                if (i != 0 && mMap.vpKeyFramessec[cn].size()>0)
                    neednewkfsec = NeedNewKeyFrame(pK2[cn]);
                else
                    neednewkfsec = true;
            }
        }
        else
            neednewkfsec = false;
        neednewkf = neednewkf || neednewkfsec;
        cout << "need new kf from secon cam?: " << neednewkfsec << endl;

//        // and every kf with a association in queuesec should correct its association number
//        for (int j = 0; j < mvpKeyFrameQueueSec.size(); j++){
//            if (mvpKeyFrameQueueSec[j]->mAssociateKeyframe && !mvpKeyFrameQueueSec[j]->mAssociatedinFinalQueue)
//                mvpKeyFrameQueueSec[j]->nAssociatedKf --;
//        }

        if (neednewkf){// add this kf!
            // in dcslam system, the association number is useless
            if (usingDualimg){
//                assert(mMap.vpKeyFrames.size() == mMap.vpKeyFramessec.size());
                pK->nAssociatedKf = mMap.vpKeyFramessec[0].size();
                for (int cn = 0; cn < AddCamNumber; cn ++)
                    pK2[cn]->nAssociatedKf = mMap.vpKeyFrames.size();
            }
//            if (pK->mAssociateKeyframe){
//                if (!pK->mAssociatedinFinalQueue){
//                    mvpKeyFrameQueueSec[pK->nAssociatedKf]->nAssociatedKf = mMap.vpKeyFrames.size();
//                    mvpKeyFrameQueueSec[pK->nAssociatedKf]->mAssociatedinFinalQueue = true;
//                }
//                else{
//                    mMap.vpKeyFramessec[pK->nAssociatedKf]->nAssociatedKf = mMap.vpKeyFrames.size();
//                    mMap.vpKeyFramessec[pK->nAssociatedKf]->mAssociatedinFinalQueue = true;
//                }
//            }
            break;
        }
//        else if (pK->mAssociateKeyframe){ // label the association relationship of its associated kf
//            if (!pK->mAssociatedinFinalQueue)
//                mvpKeyFrameQueueSec[pK->nAssociatedKf]->mAssociateKeyframe = false;
//            else
//                mMap.vpKeyFramessec[pK->nAssociatedKf]->mAssociateKeyframe = false;
//        }
    }
    lockqueue.unlock();
    cout << "Adding Keyframes preparation done..." << endl;

    if (!neednewkf)
        return;

    //    pK = mvpKeyFrameQueue[0];
    //    mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
    pK->SBI = SmallBlurryImage(*pK); // Regenerate Small Blurry Image
    pK->SBI.MakeJacs();
    if (!pK->bComplete)
        pK->MakeKeyFrame_Rest();

    if (mMap.vpKeyFrames.size() < *gvnFixedFrameSize)
        pK->bFixed = true;

    static int kfid = 1;
    pK->id = kfid;
    kfid ++;

    //  assert(pK->aLevels[0].vCandidates.size() > 0);
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);
    mMap.vpKeyFrames.push_back(pK);

    cout << "AddKeyFrame pose: \n"<< mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW <<"\n";
    // Any measurements? Update the relevant point's measurement counter status map
    for(meas_it it = pK->mMeasurements.begin();
        it!=pK->mMeasurements.end();
        it++)
    {
        it->first->MMData.sMeasurementKFs.insert(pK);
        it->second.Source = Measurement::SRC_TRACKER;
    }

//    cout << "Adding map points from first cam keyframe..." << endl;
    // And maybe we missed some - this now adds to the map itself, too.
    ReFindInSingleKeyFrame(pK);

    AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
    AddSomeMapPoints(0);
    AddSomeMapPoints(1);
    AddSomeMapPoints(2);
    cout << "Added map points from First cam keyframe..."<< mMap.vpKeyFrames.size() << endl;

//    // Remove old keyframes
//    if (*gvnVOonly!=0)
//        mMap.EraseOldKeyFrames(false);

    /////////////////////////////////////////////////////////////////////////
    // keyframes from the second camera
//    if(mvpKeyFrameQueueSec.size() == 0)
//        return;

//    for (int i = 0; i < mvpKeyFrameQueueSec.size(); i ++){
//        pK2 = mvpKeyFrameQueueSec[0];
//        mvpKeyFrameQueueSec.erase(mvpKeyFrameQueueSec.begin());

//        if (mMap.vpKeyFramessec.size()){
//            neednewkfsec = NeedNewKeyFrame(pK2, 1);
//            bool associatedadded = pK2->mAssociateKeyframe && pK2->mAssociatedinFinalQueue;
//            neednewkfsec = neednewkfsec || associatedadded;
//            // then may have second cam kfs, which have no associated first cam kfs
//            // second cam kfs: *   *   *   *   *  * *
//            // first cam kfs:  +   +   +   +      + +
//            // and not in the oppersite way, since the sec cam kfs always added when associated first cam kf is added
//            // and i forced adding new kfs to queue only when both cam kfs avilable in the tracker
//        }
//        else
//            neednewkfsec = true;

//        // and every kf with an association in queuesec should correct its association number
//        for (int j = 0; j < mvpKeyFrameQueue.size(); j++){
//            if (mvpKeyFrameQueue[j]->mAssociateKeyframe && !mvpKeyFrameQueue[j]->mAssociatedinFinalQueue)
//                mvpKeyFrameQueue[j]->nAssociatedKf --;
//        }

//        if (neednewkfsec){// add this kf! and label the association
//            if (pK2->mAssociateKeyframe){
//                if (!pK2->mAssociatedinFinalQueue){
//                    mvpKeyFrameQueue[pK2->nAssociatedKf]->nAssociatedKf = mMap.vpKeyFramessec.size();
//                    mvpKeyFrameQueue[pK2->nAssociatedKf]->mAssociatedinFinalQueue = true;
//                }
//                else{
//                    mMap.vpKeyFrames[pK2->nAssociatedKf]->nAssociatedKf = mMap.vpKeyFramessec.size();
//                    mMap.vpKeyFrames[pK2->nAssociatedKf]->mAssociatedinFinalQueue = true;
//                }
//            }
//            break;
//        }
//        else if (pK2->mAssociateKeyframe){ // label the association relationship of its associated kf
//            if (!pK2->mAssociatedinFinalQueue)
//                mvpKeyFrameQueue[pK2->nAssociatedKf]->mAssociateKeyframe = false;
//            else
//                mMap.vpKeyFrames[pK2->nAssociatedKf]->mAssociateKeyframe = false;
//        }
//    }

    if (!usingDualimg){
        lock.unlock();
        mbBundleConverged_Full = false;
        mbBundleConverged_Recent = false;
        newRecentKF= true;
        if (*gvnVOonly!=0)// Remove old keyframes
            mMap.EraseOldKeyFrames(false);
        return;
    }

    for (int cn = 0; cn < AddCamNumber; cn ++){
        pK2[cn]->SBI = SmallBlurryImage(*pK2[cn]); // Regenerate Small Blurry Image
        pK2[cn]->SBI.MakeJacs();
        if (!pK2[cn]->bComplete)
            pK2[cn]->MakeKeyFrame_Rest();

        // try also fix the second kf, since the ini map is always accurate till now
        if (mMap.vpKeyFramessec[cn].size() < *gvnFixedFrameSize)
            pK2[cn]->bFixed = true;

        // id depends on whether add kf when ini by circle
        pK2[cn]->id = secKFid[cn];
        secKFid[cn] ++;
        mMap.vpKeyFramessec[cn].push_back(pK2[cn]);
        // Any measurements? Update the relevant point's measurement counter status map
        for(meas_it it = pK2[cn]->mMeasurements.begin();
            it!=pK2[cn]->mMeasurements.end();
            it++)
        {
            it->first->MMData.sMeasurementKFs.insert(pK2[cn]);
            it->second.Source = Measurement::SRC_TRACKER;
        }
        // different camera model for the second camera keyframes,
        // mappoints should only be triangulated among those kfs from the same camera
        if(mMap.vpKeyFramessec[cn].size() < 2)
            return;
        // And maybe we missed some - this now adds to the map itself, too.
        ReFindInSingleKeyFrame(pK2[cn]);//

        cout << "Adding map points from sec cam keyframe..." << endl;
        AddSomeMapPoints(3, cn);       // .. and add more map points by epipolar search.
        AddSomeMapPoints(0, cn);
        AddSomeMapPoints(1, cn);
        AddSomeMapPoints(2, cn);
        cout << "Added map points from sec cam keyframe..."<< mMap.vpKeyFramessec[cn].size() << endl;
    }

    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;
//    if (mMap.vpKeyFramessec.size()<4)// bundle adjust the first three kf, execpt the first one.
//        BundleAdjustAllsec();

    nAddedKfNoBA ++;
    lock.unlock();
    newRecentKF= true;
    if (*gvnVOonly!=0)// Remove old keyframes
        mMap.EraseOldKeyFrames(true);

//    std::cout<<"associations: " << "\n";
//    for (int i = 0; i < mMap.vpKeyFramessec.size(); i ++)
//        std::cout << mMap.vpKeyFramessec[i]->nAssociatedKf << ", ";
//    std::cout << std::endl;
//    for (int i = 0; i < mMap.vpKeyFrames.size(); i ++)
//        std::cout << mMap.vpKeyFrames[i]->nAssociatedKf << ", ";
//    std::cout << std::endl;
}

bool MapMaker::AddPointDepth(boost::shared_ptr<KeyFrame> kSrc,
                             boost::shared_ptr<KeyFrame> kTarget,
                             int nLevel,
                             int nCandidate
                             )
{
    int nCam = kSrc->nSourceCamera;
    int nLevelScale = LevelScale(nLevel);
    Candidate &candidate = kSrc->aLevels[nLevel].vCandidates[nCandidate];
    const ImageRef& irLevelPos = candidate.irLevelPos;
    Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);

    // project candidate into target frame
    Vector<3> v3Unprojected;
    if (!nCam)
        v3Unprojected = unproject(mCamera->UnProject(v2RootPos));
    else
        v3Unprojected = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos));
    Vector<3> v3PosWorld = kSrc->se3CfromW.inverse()*(candidate.dDepth*v3Unprojected);
    Vector<3> v3PosTarget = kTarget->se3CfromW*v3PosWorld;
    ImageRef irTarget;
    if (!nCam)
        irTarget = ir(mCamera->Project(project(v3PosTarget)));
    else
        irTarget = ir(mCameraSec[nCam - 1]->Project(project(v3PosTarget)));

    // Find current-frame corners which might match this
    PatchFinder Finder;
    Finder.MakeTemplateCoarseNoWarp(*kSrc, nLevel, irLevelPos);
    if(Finder.TemplateBad())  return false;

    boost::shared_ptr<MapPoint> pNew(new MapPoint);
    pNew->v3WorldPos = v3PosWorld;

    pNew->v3SourceKFfromeWorld = kSrc->se3CfromW;
    pNew->v3RelativePos = kSrc->se3CfromW * pNew->v3WorldPos; //(not used since father-relation may be transfered)

    // Patch source stuff:
    pNew->pPatchSourceKF = kSrc;
    pNew->nSourceLevel = nLevel;
    pNew->v3Normal_NC = makeVector( 0,0,-1);
    pNew->irCenter = irLevelPos;

    if (!nCam){
        pNew->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
        pNew->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
        pNew->v3OneDownFromCenter_NC  = unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
    }
    else{
        pNew->v3Center_NC = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos));
        pNew->v3OneRightFromCenter_NC = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
        pNew->v3OneDownFromCenter_NC  = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
    }

    normalize(pNew->v3Center_NC);
    normalize(pNew->v3OneDownFromCenter_NC);
    normalize(pNew->v3OneRightFromCenter_NC);

    pNew->nSourceCamera = nCam;
    pNew->bfixed = kSrc->bFixed&&kTarget->bFixed;

    pNew->RefreshPixelVectors();

    mMap.vpPoints.push_back(pNew);
    mqNewQueue.push(pNew);
    Measurement m;
    m.Source = Measurement::SRC_ROOT;
    m.v2RootPos = v2RootPos;
    m.dDepth = candidate.dDepth;
    m.nLevel = nLevel;
    m.bSubPix = true;
    kSrc->mMeasurements[pNew] = m;
    pNew->MMData.sMeasurementKFs.insert(kSrc);

    if (Finder.FindPatchCoarse(irTarget,*kTarget,10)) {
        Finder.MakeSubPixTemplate();
        Finder.SetSubPixPos(Finder.GetCoarsePosAsVector());
        if (Finder.IterateSubPixToConvergence(*kTarget,10)) {
            m.Source = Measurement::SRC_EPIPOLAR;
            m.v2RootPos = Finder.GetSubPixPos();
            m.dDepth = Finder.GetCoarseDepth();
            m.nLevel = Finder.GetLevel();
            //m.bSubPix = true;
            kTarget->mMeasurements[pNew] = m;
            pNew->MMData.sMeasurementKFs.insert(kTarget);
        }
    }

    return true;
}

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(boost::shared_ptr<KeyFrame> kSrc, 
                                boost::shared_ptr<KeyFrame> kTarget,
                                int nLevel,
                                int nCandidate)
{
    int nCam = kSrc->nSourceCamera;
    static Image<Vector<2> > imUnProj;
    static Image<Vector<2> > imUnProjsec;// look out this static member!!
    static bool bMadeCache = false;
    static bool bMadeCachesec = false;
    if(!bMadeCache && !nCam)
    {
        ImageRef ir;
        imUnProj.resize(kSrc->aLevels[0].im.size());
        do imUnProj[ir] = mCamera->UnProject(ir);
        while(ir.next(imUnProj.size()));
        bMadeCache = true;
    }
    else if(!bMadeCachesec && nCam)
    {
        ImageRef ir;
        imUnProjsec.resize(kSrc->aLevels[0].im.size());
        do imUnProjsec[ir] = mCameraSec[nCam - 1]->UnProject(ir);
        while(ir.next(imUnProjsec.size()));
        bMadeCachesec = true;
    }

    int nLevelScale = LevelScale(nLevel);
    Candidate &candidate = kSrc->aLevels[nLevel].vCandidates[nCandidate];
    ImageRef irLevelPos = candidate.irLevelPos;
    Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);

    Vector<3> v3Ray_SC;
    if (!nCam){
        v3Ray_SC = unproject(mCamera->UnProject(v2RootPos));
    }
    else
    {
        v3Ray_SC = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos));
    }
    normalize(v3Ray_SC);
    assert(v3Ray_SC[2] >= 0);
    Vector<3> v3LineDirn_TC = kTarget->se3CfromW.get_rotation() * (kSrc->se3CfromW.get_rotation().inverse() * v3Ray_SC);
    // Restrict epipolar search to a relatively narrow depth range
    // to increase reliability
    // dual img case: no scenedepth information before new mappoints been added.
    // so depth should not be limited if dSceneDepthMean=initial value.
    double dMean = kSrc->dSceneDepthMean;
    double dSigma = kSrc->dSceneDepthSigma;
    double dStartDepth;
    double dEndDepth;
    static gvar3<double> gvndSceneDepthMaxSecCam("MapMaker.SceneDepthMaxSecCam", 5.0, SILENT);
    static gvar3<double> gvndSceneDepthMinSecCam("MapMaker.SceneDepthMinSecCam", 2.0, SILENT);
    if ((kSrc->dSceneDepthMean==1.0) && (kSrc->dSceneDepthSigma==1.0))// weak constrain for initial case
    {
        dStartDepth = *gvndSceneDepthMinSecCam;
        dEndDepth = *gvndSceneDepthMaxSecCam;
//        cout << "depth not initialised "<< dMean << ", " << dSigma << endl;
    }
    else
    {
        dStartDepth = max(mdWiggleScale, dMean - dSigma);
        dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);
    }

    Vector<3> v3CamCenter_TC = kTarget->se3CfromW * kSrc->se3CfromW.inverse().get_translation(); // The camera end
    Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
    Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end
    //  cout << "start and end: " << v3LineDirn_TC[0] << ", " << v3LineDirn_TC[1] << ", "
    //       << v3LineDirn_TC[2]<< ", " << dMean << ", " << dSigma << ", " << dStartDepth << ", " << dEndDepth << endl;

    if(v3RayEnd_TC[2] <= v3RayStart_TC[2])  // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
        return false;
    if(v3RayStart_TC[2] <= 0.0 )  return false;
    // yang, ignore those behind
//    if(v3RayStart_TC[2] <= 0.0)
//    v3RayStart_TC[2] = std::max(v3RayStart_TC[2], 0.0);
//    if(v3RayStart_TC[2] <= 0.0)
//        v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);

    Vector<2> v2A = project(v3RayStart_TC);
    Vector<2> v2B = project(v3RayEnd_TC);
    Vector<2> v2AlongProjectedLine = v2A-v2B;

    //  cout << "search depth: " << v2AlongProjectedLine[0] << ",  " << v2AlongProjectedLine[1] << endl;
    if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
    {
        cout << "v2AlongProjectedLine too small." << endl;
        return false;
    }
    normalize(v2AlongProjectedLine);
    Vector<2> v2Normal;
    v2Normal[0] = v2AlongProjectedLine[1];
    v2Normal[1] = -v2AlongProjectedLine[0];

    double dNormDist = v2A * v2Normal;
    if (!nCam){
        if(fabs(dNormDist) > mCamera->LargestRadiusInImage() )
            return false;
    }
    else{
        if(fabs(dNormDist) > mCameraSec[nCam - 1]->LargestRadiusInImage() )
            return false;
    }

    double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
    double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
    if(dMinLen < -2.0)  dMinLen = -2.0;
    if(dMaxLen < -2.0)  dMaxLen = -2.0;
    if(dMinLen > 2.0)   dMinLen = 2.0;
    if(dMaxLen > 2.0)   dMaxLen = 2.0;

    // Find current-frame corners which might match this
    PatchFinder Finder;
    Finder.MakeTemplateCoarseNoWarp(*kSrc, nLevel, irLevelPos);
    if(Finder.TemplateBad())  return false;

    vector<Vector<2> > &vv2Corners = kTarget->aLevels[nLevel].vImplaneCorners;
    vector<ImageRef> &vIR = kTarget->aLevels[nLevel].vCorners;
    if(!kTarget->aLevels[nLevel].bImplaneCornersCached)
    {
        for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
        {
            if (!nCam)
                vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
            else
                vv2Corners.push_back(imUnProjsec[ir(LevelZeroPos(vIR[i], nLevel))]);
        }
        kTarget->aLevels[nLevel].bImplaneCornersCached = true;
    }

    int nBest = -1;
    int nBestZMSSD = Finder.mnMaxSSD + 1;
    double dMaxDistDiff;
    if (!nCam){
        dMaxDistDiff = mCamera->OnePixelDist() * (4.0 + 1.0 * nLevelScale);
    }
    else{
        dMaxDistDiff = mCameraSec[nCam - 1]->OnePixelDist() * (4.0 + 1.0 * nLevelScale);
    }
    double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

    for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
    {
        Vector<2> v2Im = vv2Corners[i];
        double dDistDiff = dNormDist - v2Im * v2Normal;
        if(dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
        if(v2Im * v2AlongProjectedLine < dMinLen)	continue; // skip if not far enough along line
        if(v2Im * v2AlongProjectedLine > dMaxLen)	continue; // or too far
        int nZMSSD = Finder.ZMSSDAtPoint(kTarget->aLevels[nLevel].im, vIR[i]);
        if(nZMSSD < nBestZMSSD)
        {
            nBest = i;
            nBestZMSSD = nZMSSD;
        }
    }
    if(nBest == -1)  return false;   // Nothing found.

    //  Found a likely candidate along epipolar ray
    Finder.MakeSubPixTemplate();
    Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
    bool bSubPixConverges = Finder.IterateSubPixToConvergence(*kTarget,10);
    if(!bSubPixConverges)
        return false;

    // Now triangulate the 3d point...
    // yang, ignore those points whose two view projections have a very similar direction in the world frame
    static gvar3<double> minViewAngleDiff("MapMaker.minViewAngleDiff", 3.0, SILENT);
    Vector<3> v3New;
    if (!nCam){
        SE3<> tc12 = kSrc->se3CfromW * kTarget->se3CfromW.inverse();
        Vector<2> nc1, nc2;
        nc1 = mCamera->UnProject(v2RootPos);
        nc2 = mCamera->UnProject(Finder.GetSubPixPos());
        Vector<3> tc1, tc2;
        tc1 = unproject(nc1);
        tc2 = unproject(nc2);
        double viewangle = viewAngleDiffPoint(tc12.inverse().get_rotation()*tc1, tc2);
        double mindiff = *minViewAngleDiff *2*3.14/180.0;
        if (viewangle < mindiff)
            return false;

        Vector<3> reprop = ReprojectPoint(tc12, nc1, nc2);
        if (reprop[2] <= std::max(0.0, tc12.inverse().get_translation()[2]))
            return false;

        v3New = kTarget->se3CfromW.inverse() * reprop;
//        pos_log_ << "cam0 view angle: " << viewangle*180/3.14 << ", " << reprop
//                 << ", " << tc12.get_translation() << ", "
//                 <<tc12.inverse().get_rotation()*tc1<<", "<< tc2<< endl;
    }
    else{
//        cout << "cam 1 ksc ktarget pixel: " << v2RootPos[0] << ", " << v2RootPos[1] << "  "
//             << Finder.GetSubPixPos()[0] << ", " << Finder.GetSubPixPos()[1] << endl;
        SE3<> tc12 = kSrc->se3CfromW * kTarget->se3CfromW.inverse();
        Vector<2> nc1, nc2;
        nc1 = mCameraSec[nCam - 1]->UnProject(v2RootPos);
        nc2 = mCameraSec[nCam - 1]->UnProject(Finder.GetSubPixPos());
        Vector<3> tc1, tc2;
        tc1 = unproject(nc1);
        tc2 = unproject(nc2);
        double viewangle = viewAngleDiffPoint(tc12.inverse().get_rotation()*tc1, tc2);
        double mindiff = *minViewAngleDiff *3.14/180.0;
        if (viewangle < mindiff)
            return false;

        Vector<3> reprop = ReprojectPoint(tc12, nc1, nc2);
        if (reprop[2] <= std::max(0.0, tc12.inverse().get_translation()[2]))
            return false;

        v3New = kTarget->se3CfromW.inverse() * reprop;
//        pos_log_ << "cam1 view angle: " << viewangle*180/3.14 << ", " << reprop
//                 << ", " <<tc12.inverse().get_rotation()*tc1<<", "<< tc2<< endl;
    }

    boost::shared_ptr<MapPoint> pNew(new MapPoint);
    pNew->v3WorldPos = v3New;

    pNew->v3SourceKFfromeWorld = kSrc->se3CfromW;
    pNew->v3RelativePos = kSrc->se3CfromW * pNew->v3WorldPos; //(not used since father-relation may be transfered)

    // Patch source stuff:
    pNew->pPatchSourceKF = kSrc;
    pNew->nSourceLevel = nLevel;
    pNew->v3Normal_NC = makeVector( 0,0,-1);
    pNew->irCenter = irLevelPos;
    if (!nCam){
        pNew->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
        pNew->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
        pNew->v3OneDownFromCenter_NC  = unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
    }
    else{
        pNew->v3Center_NC = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos));
        pNew->v3OneRightFromCenter_NC = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
        pNew->v3OneDownFromCenter_NC  = unproject(mCameraSec[nCam - 1]->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
    }

    normalize(pNew->v3Center_NC);
    normalize(pNew->v3OneDownFromCenter_NC);
    normalize(pNew->v3OneRightFromCenter_NC);

    pNew->RefreshPixelVectors();

    pNew->nSourceCamera = nCam;
    //if (!nCam)
    pNew->bfixed = kSrc->bFixed&&kTarget->bFixed;
    
    mMap.vpPoints.push_back(pNew);
    mqNewQueue.push(pNew);
    Measurement m;
    m.dDepth = 0;
    m.Source = Measurement::SRC_ROOT;
    m.v2RootPos = v2RootPos;
    m.nLevel = nLevel;
    m.bSubPix = true;
    kSrc->mMeasurements[pNew] = m;

    m.Source = Measurement::SRC_EPIPOLAR;
    m.v2RootPos = Finder.GetSubPixPos();
    kTarget->mMeasurements[pNew] = m;
    pNew->MMData.sMeasurementKFs.insert(kSrc);
    pNew->MMData.sMeasurementKFs.insert(kTarget);
    return true;
}

double MapMaker::KeyFrameDist(KeyFrame &k1, KeyFrame &k2)
{
    static gvar3<double> gvnLinearDist("MapMaker.LinearDist", 0.2, SILENT);
    static gvar3<double> gvnAngularDist("MapMaker.AngularDist", (20.0*M_PI/180.0), SILENT);
//    cout << "kf dist: " << KeyFrameLinearDist(k1,k2)<< "/"<< (*gvnLinearDist) << ", " << KeyFrameAngularDist(k1,k2)<< "/"<<(*gvnAngularDist) << endl;
    return KeyFrameLinearDist(k1,k2)/(*gvnLinearDist) + KeyFrameAngularDist(k1,k2)/(*gvnAngularDist);
}

double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
    Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
    Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
    Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
    double dDist = sqrt(v3Diff * v3Diff);
    return dDist;
}

double MapMaker::KeyFrameAngularDist(KeyFrame &k1, KeyFrame &k2)
{
    SE3<> se3C2fromC1 = k2.se3CfromW*k1.se3CfromW.inverse();
    Matrix<3,3> R = se3C2fromC1.get_rotation().get_matrix();
    double trace = R(0,0) + R(1,1) + R(2,2);
//    cout << "k1: " << k1.se3CfromW<< endl;
//    cout << "k2: " << k2.se3CfromW<< endl;
//    cout << "R: " << R<< endl;
//    cout << "trace: " << (trace-1.0)/2 << endl;
//    assert(abs((trace-1.0)/2)<=1.0);
    double cosR= (trace-1.0)/2.0;
    if (cosR < -1.0)
        cosR = -1.0;
    else if (cosR > 1.0)
        cosR = 1.0;
    return std::acos(cosR);
}

vector<boost::shared_ptr<KeyFrame> > MapMaker::NClosestKeyFrames(boost::shared_ptr<KeyFrame> k, unsigned int N,
                                                                 int nCam)
{
    assert(k->nSourceCamera == nCam);
    vector<pair<double, boost::shared_ptr<KeyFrame> > > vKFandScores;
    if (!nCam){
        for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        {
            if(mMap.vpKeyFrames[i] == k)
                continue;
            double dDist = KeyFrameLinearDist(*k, *mMap.vpKeyFrames[i]);
            vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
        }
    }
    else {
        for(unsigned int i=0; i<mMap.vpKeyFramessec[nCam-1].size(); i++)
        {
            if(mMap.vpKeyFramessec[nCam-1][i] == k)
                continue;
            double dDist = KeyFrameLinearDist(*k, *mMap.vpKeyFramessec[nCam-1][i]);
            vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFramessec[nCam-1][i]));
        }
    }
    if(N > vKFandScores.size())
        N = vKFandScores.size();
    partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

    vector<boost::shared_ptr<KeyFrame> > vResult;
    for(unsigned int i=0; i<N; i++)
        vResult.push_back(vKFandScores[i].second);
    return vResult;
}

boost::shared_ptr<KeyFrame> MapMaker::ClosestKeyFrame(boost::shared_ptr<KeyFrame> k, double mindist)
{
    int nCam = k->nSourceCamera;
    double dClosestDist = 9999999999.9;
    int nClosest = -1;
    int numkf = 0;
    if (!nCam)
        numkf = mMap.vpKeyFrames.size();
    else
        numkf = mMap.vpKeyFramessec[nCam - 1].size();
//    if (!nCam)
//        cout<< "mapping nClosest vpKeyFrames.size: " << mMap.vpKeyFrames.size()<<endl;
//    else
//        cout<< "mapping nClosest vpKeyFrames sec.size: " << mMap.vpKeyFramessec.size()<<endl;
    for(unsigned int i=0; i<numkf; i++)
    {
        double dDist = 0;
        if (!nCam){
            if(mMap.vpKeyFrames[i] == k)
                continue;
            dDist = KeyFrameDist(*k, *mMap.vpKeyFrames[i]);
        }
        else{
            if(mMap.vpKeyFramessec[nCam - 1][i] == k)
                continue;
            dDist = KeyFrameDist(*k, *mMap.vpKeyFramessec[nCam - 1][i]);
        }
        if(dDist < dClosestDist && dDist > mindist)
        {
            dClosestDist = dDist;
            nClosest = i;
        }
    }
    assert(nClosest != -1);
    if (!nCam)
        return mMap.vpKeyFrames[nClosest];
    else
        return mMap.vpKeyFramessec[nCam - 1][nClosest];
}

double MapMaker::DistToNearestKeyFrame(boost::shared_ptr<KeyFrame> kCurrent)
{
    boost::shared_ptr<KeyFrame> pClosest = ClosestKeyFrame(kCurrent);
    double dDist = KeyFrameLinearDist(*kCurrent, *pClosest);
    return dDist;
}

// yang, original PTAM problem: when mapping thread is busy, it may not be able to add
// a keyframe in time. Then tracking thread may add too many very close keyframes to the
// waiting list, if the quadrotor flys fast.
bool MapMaker::NeedNewKeyFrame(boost::shared_ptr<KeyFrame> kCurrent)
{
//    std::cout << "calculating the closest kf... " << endl;
    boost::shared_ptr<KeyFrame> pClosest = ClosestKeyFrame(kCurrent,0.0);
//    std::cout << "Closest kf id: " << pClosest->id<< ", latest kfs id: " << mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->id << endl;
//      std::cout << "dist to closest keyframe: linear " << KeyFrameLinearDist(*kCurrent,*pClosest)
//                << ", angular: " << KeyFrameAngularDist(*kCurrent,*pClosest) << std::endl;
//      std::cout << "total dist and cur pose: " << KeyFrameDist(*kCurrent,*pClosest) << " \n"
//                << kCurrent->se3CfromW << std::endl;
//      std::cout << "dist to last kf: linear " << KeyFrameLinearDist(*kCurrent,*mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1])
//                << ", angular: " << KeyFrameAngularDist(*kCurrent,*mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]) << std::endl;
//      std::cout << "last kf pose: \n" << mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW << std::endl;

    bool neednew = false;

    //// TODO: distance threshold should be able to be configured from conf. file
//    if ((kCurrent->se3CfromW.inverse().get_translation()[2]<0.3
//         && KeyFrameDist(*kCurrent, *pClosest)>0.2)
//            || (kCurrent->se3CfromW.inverse().get_translation()[2]<0.4
//                && KeyFrameDist(*kCurrent, *pClosest)>0.4)
//            || (kCurrent->se3CfromW.inverse().get_translation()[2]<0.6
//                && KeyFrameDist(*kCurrent, *pClosest)>0.6)
//            || (kCurrent->se3CfromW.inverse().get_translation()[2]<1.0
//                && KeyFrameDist(*kCurrent, *pClosest)> 0.8*kCurrent->se3CfromW.inverse().get_translation()[2])
//            || (kCurrent->se3CfromW.inverse().get_translation()[2]>1.0
//                &&KeyFrameDist(*kCurrent, *pClosest) > 0.9*kCurrent->se3CfromW.inverse().get_translation()[2]))
        if ( KeyFrameDist(*kCurrent, *pClosest)>0.5)
        neednew = true;
    return neednew;
    //  return KeyFrameDist(*kCurrent,*pClosest) > 0.2;//yang, 1.0;
    //  dDist *= (1.0 / kCurrent.dSceneDepthMean);

    //  if(dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdWiggleScaleDepthNormalized)
    //    return true;
    //  return false;
}

// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll()
{
    // construct the sets of kfs/points to be adjusted:
    // in this case, all of them
    set<boost::shared_ptr<KeyFrame> > sAdj;
    set<boost::shared_ptr<KeyFrame> > sFixed;
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        if(mMap.vpKeyFrames[i]->bFixed)
            sFixed.insert(mMap.vpKeyFrames[i]);
        else
            sAdj.insert(mMap.vpKeyFrames[i]);

    set<boost::shared_ptr<MapPoint> > sMapPoints;
    for(unsigned int i=0; i<mMap.vpPoints.size();i++)
        sMapPoints.insert(mMap.vpPoints[i]);

    BundleAdjust(sAdj, sFixed, sMapPoints, false);
}
// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAllsec()
{
    // construct the sets of kfs/points to be adjusted:
    // in this case, all of them
    set<boost::shared_ptr<KeyFrame> > sAdj;
    set<boost::shared_ptr<KeyFrame> > sFixed;

    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        if(mMap.vpKeyFrames[i]->bFixed)
            sFixed.insert(mMap.vpKeyFrames[i]);
        else
            sAdj.insert(mMap.vpKeyFrames[i]);
    for (int cn = 0; cn < AddCamNumber; cn ++)
      for(unsigned int i=0; i<mMap.vpKeyFramessec[cn].size(); i++){
        if(mMap.vpKeyFramessec[cn][i]->bFixed)
            sFixed.insert(mMap.vpKeyFramessec[cn][i]);
        else
            sAdj.insert(mMap.vpKeyFramessec[cn][i]);
    }

    set<boost::shared_ptr<MapPoint> > sMapPoints;
    for(unsigned int i=0; i<mMap.vpPoints.size();i++)
//        if (mMap.vpPoints[i]->nSourceCamera)// sec img
            sMapPoints.insert(mMap.vpPoints[i]);

    BundleAdjust(sAdj, sFixed, sMapPoints, false);
}
// Peform a local bundle adjustment which only adjusts
// recently added key-frames
// * remember to treat mappoints and keyframes from dual cameras separately
void MapMaker::BundleAdjustRecent()
{
    static gvar3<int> gvnWindowSize("MapMaker.RecentWindowSize", 4, SILENT);
    if(mMap.vpKeyFrames.size() < *gvnWindowSize)
    { // Ignore this unless map is big enough
        mbBundleConverged_Recent = true;
        return;
    }

    // First, make a list of the keyframes we want adjusted in the adjuster.
    // This will be the last keyframe inserted, and its four nearest neighbors
    // The X set in the original paper PTAM
    // *for the dual img case, also for those kfs of the second img
    // *kfs from the second camera are already labeled with their camera number,
    // which will be useful for reprojecting their associated mappoints
    set<boost::shared_ptr<KeyFrame> > sAdjustSet;
    set<boost::shared_ptr<KeyFrame> > sAssociatedSet;// KFs should be adjusted but not added to adjustset due to association with kfs from first cam
                                                     // we only adjust their associated kfs from the master cam
    boost::shared_ptr<KeyFrame> pkfNewest = mMap.vpKeyFrames.back();
    sAdjustSet.insert(pkfNewest);

    static gvar3<int> gvnOrderByTime("MapMaker.OrderByTime", 1, SILENT);
    static gvar3<int> gvnFixedKFinLBA("MapMaker.FixedKFinLBA", 2, SILENT);

    // TODO: to find out why here cannot order by distance
    if (*gvnOrderByTime == 0){// order by distance
        vector<boost::shared_ptr<KeyFrame> > vClosest = NClosestKeyFrames(pkfNewest, *gvnWindowSize);
        for(int i = 0; i < std::min(*gvnWindowSize,(int) vClosest.size()); i++)
            if(vClosest[i]->bFixed == false)
                sAdjustSet.insert(vClosest[i]);
    }
    else{// order just by time
        boost::shared_ptr<KeyFrame> lastkf;
        for(int i = mMap.vpKeyFrames.size()-2; i >=std::max((int)mMap.vpKeyFrames.size()-*gvnWindowSize,*gvnFixedKFinLBA) ; i--){
            lastkf = mMap.vpKeyFrames[i];
            if(lastkf->bFixed == false)
                sAdjustSet.insert(lastkf);
        }
    }

    static gvar3<int> gvnSecondCamIndependent("MapMaker.SecondCamIndependent", 0, SILENT);
    // second camera kfs, closest kfs. if associated with first cam kfs, label them, which will not be adjusted seperately
    for (int cn = 0; cn < AddCamNumber; cn ++){
        if (mMap.vpKeyFramessec[cn].size() >= (unsigned int) (*gvnWindowSize)
                && !mMap.vpKeyFramessec[cn].back()->bFixed)
        {
            pkfNewest = mMap.vpKeyFramessec[cn].back();
            if (!pkfNewest->mAssociateKeyframe)// independent kfs. if associated, its pose will be rigidly related to the first cam pose
                sAdjustSet.insert(pkfNewest);  // We don't add those kfs from the second cam, even if their associated kf from first cam are not added
            else /// most likely, if with association
            {
                if (*gvnSecondCamIndependent == 0){
                    //                sAdjustSet.insert(pkfNewest);
                    //            else
                    sAssociatedSet.insert(pkfNewest);
                    if(!sAdjustSet.count(mMap.vpKeyFrames[pkfNewest->nAssociatedKf]))
                        sAdjustSet.insert(mMap.vpKeyFrames[pkfNewest->nAssociatedKf]);
                }
                else{// two cameras are independent
                    sAdjustSet.insert(pkfNewest);
                    //                    sAssociatedSet.insert(pkfNewest);
                }
            }

            if (*gvnOrderByTime == 0){
                vector<boost::shared_ptr<KeyFrame> > vClosest = NClosestKeyFrames(pkfNewest, *gvnWindowSize, 1);
                for(int i = 0; i < std::min(*gvnWindowSize,(int) vClosest.size()); i++)
                    if(vClosest[i]->bFixed == false){
                        if (!vClosest[i]->mAssociateKeyframe)
                        {
                            sAdjustSet.insert(vClosest[i]);
                            //                    independnum ++;
                        }
                        else {
                            if (*gvnSecondCamIndependent == 0){
                                //                sAdjustSet.insert(pkfNewest);
                                //            else
                                sAssociatedSet.insert(vClosest[i]);
                                if (!sAdjustSet.count(mMap.vpKeyFrames[vClosest[i]->nAssociatedKf]))
                                    sAdjustSet.insert(mMap.vpKeyFrames[vClosest[i]->nAssociatedKf]);

                            }
                            else{
                                sAdjustSet.insert(vClosest[i]);
                                //                                sAssociatedSet.insert(vClosest[i]);
                            }
                        }
                    }
            }
            else{
                boost::shared_ptr<KeyFrame> lastkf;
                for(int i = mMap.vpKeyFramessec[cn].size()-2; i >=std::max((int)mMap.vpKeyFramessec[cn].size()-*gvnWindowSize,*gvnFixedKFinLBA); i--){
                    lastkf = mMap.vpKeyFramessec[cn][i];
                    if(lastkf->bFixed == false){
                        if (!lastkf->mAssociateKeyframe)
                        {
                            sAdjustSet.insert(lastkf);
                            //                    independnum ++;
                        }
                        else {
                            if (*gvnSecondCamIndependent == 0){
                                //                sAdjustSet.insert(pkfNewest);
                                //            else
                                sAssociatedSet.insert(lastkf);
                                if (!sAdjustSet.count(mMap.vpKeyFrames[lastkf->nAssociatedKf]))
                                    sAdjustSet.insert(mMap.vpKeyFrames[lastkf->nAssociatedKf]);
                            }
                            else{
                                sAdjustSet.insert(lastkf);
                                //                                sAssociatedSet.insert(lastkf);
                            }
                        }
                    }
                }
            }
        }
    }
//    cout << "independent and associated second cam kf num: " <<independnum << ", "<< sAssociatedSet.size() << endl;

    // Now we find the set of features which they contain.
//    int addassociate = 0;
    set<boost::shared_ptr<MapPoint> > sMapPoints;
    for(set<boost::shared_ptr<KeyFrame> >::iterator iter = sAdjustSet.begin();
        iter!=sAdjustSet.end();
        iter++)
    {
        map<boost::shared_ptr<MapPoint> ,Measurement> &mKFMeas = (*iter)->mMeasurements;
        for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
//            if (!jiter->first->bfixed)
                sMapPoints.insert(jiter->first);

        // bug fixed! I should never re-assign a reference type value!!!!
        // also add those mappoints from the associated second cam kfs
//        if ((*gvnSecondCamIndependent == 0)
//                &&!(*iter)->nSourceCamera && (*iter)->mAssociateKeyframe
//                && sAssociatedSet.count(mMap.vpKeyFramessec[(*iter)->nAssociatedKf]))
//        {
//            mKFMeas = mMap.vpKeyFramessec[(*iter)->nAssociatedKf]->mMeasurements;
//            for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
//                //                        if (!jiter->first->bfixed)
//                sMapPoints.insert(jiter->first);
//            addassociate ++;
//        }
    };
    // also add those mappoints from the associated second cam kfs
    for(set<boost::shared_ptr<KeyFrame> >::iterator iter = sAssociatedSet.begin();
        iter!=sAssociatedSet.end();
        iter++)
    {
        map<boost::shared_ptr<MapPoint> ,Measurement> &mKFMeas = (*iter)->mMeasurements;
        for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
//            if (!jiter->first->bfixed)
            sMapPoints.insert(jiter->first);
//        addassociate ++;
    }
//    cout << "Added associated second cam kf num: " <<addassociate << endl;
//    pos_log_ << "Added associated kfs: " << sAssociatedSet.size() << ", " << addassociate << endl;

    // Finally, add all keyframes which measure above points as fixed keyframes
    // *the Y set in the original paper PTAM
    // in DCSLAM, those kfs are the oldest two? keyframes, which
    // are Implicitly set to be fixed kfs.
    set<boost::shared_ptr<KeyFrame> > sFixedSet;
    for(vector<boost::shared_ptr<KeyFrame> >::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
        if(sAdjustSet.count(*it))
            continue;
        bool bInclude = false;
        for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
            if(sMapPoints.count(jiter->first))
            {
                bInclude = true;
                break;
            }
        if(bInclude)
            sFixedSet.insert(*it);
    }
    // and keyframes from the second camera
    for (int cn = 0; cn < AddCamNumber; cn ++)
        for(vector<boost::shared_ptr<KeyFrame> >::iterator it = mMap.vpKeyFramessec[cn].begin(); it!=mMap.vpKeyFramessec[cn].end(); it++)
        {
            if(sAdjustSet.count(*it) || sAssociatedSet.count(*it))
                //                || !sFixedSet.count(mMap.vpKeyFrames[(*it)->nAssociatedKf]))
                continue;
            bool bInclude = false;
            for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
                if(sMapPoints.count(jiter->first))
                {
                    bInclude = true;
                    break;
                }
            if(bInclude)
                sFixedSet.insert(*it);
        }
    cout << "LBA prepared." << endl;

    BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true, sAssociatedSet);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjust(set<boost::shared_ptr<KeyFrame> > sAdjustSet, set<boost::shared_ptr<KeyFrame> > sFixedSet, set<boost::shared_ptr<MapPoint> > sMapPoints, bool bRecent, std::set<boost::shared_ptr<KeyFrame> > sAssociatedSet)
{
    Bundle b;   // Our bundle adjuster
    mbBundleRunning = true;
    mbBundleRunningIsRecent = bRecent;
    for (int i = 0; i < AddCamNumber; i ++)
        if (mMap.vpKeyFramessec[i].size())
        b.Load_Cam2FromCam1(mse3Cam2FromCam1[i], i);

    static gvar3<int> gvnUse3D("Bundler.Use3D", 0, SILENT);
    static gvar3<int> gvnUseDepth("Bundler.UseDepth", 1, SILENT);
    static gvar3<int> gvnSecondCamIndependent("MapMaker.SecondCamIndependent", 0, SILENT);

    // The bundle adjuster does different accounting of keyframes and map points;
    // Translation maps are stored:
    map<boost::shared_ptr<MapPoint>, int> mPoint_BundleID;
    map<int, boost::shared_ptr<MapPoint> > mBundleID_Point;
    map<boost::shared_ptr<KeyFrame>, int> mView_BundleID;
    map<int, boost::shared_ptr<KeyFrame> > mBundleID_View;

    // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
    // yang, kf from the the second camera should be integrated into the related master camera kf to form a homo node
    // and update the pose of the multiply cameras assuming a fixed rig
    for(set<boost::shared_ptr<KeyFrame> >::iterator it = sAdjustSet.begin(); it!= sAdjustSet.end(); it++)
    {
        int nBundleID = b.AddCamera((*it)->se3CfromW, (*it)->bFixed, (*it)->se3Cam2fromCam1);
        mView_BundleID[*it] = nBundleID;
        mBundleID_View[nBundleID] = *it;
    }
    for(set<boost::shared_ptr<KeyFrame> >::iterator it = sFixedSet.begin(); it!= sFixedSet.end(); it++)
    {
        int nBundleID = b.AddCamera((*it)->se3CfromW, true, (*it)->se3Cam2fromCam1);
        mView_BundleID[*it] = nBundleID;
        mBundleID_View[nBundleID] = *it;
    }

    // Add the points' 3D position
    for(set<boost::shared_ptr<MapPoint> >::iterator it = sMapPoints.begin(); it!=sMapPoints.end(); it++)
    {
        int nBundleID = b.AddPoint((*it)->v3WorldPos, (*it)->bfixed);
        mPoint_BundleID[*it] = nBundleID;
        mBundleID_Point[nBundleID] = *it;
    }

    // Add the relevant point-in-keyframe measurements
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
        if(mView_BundleID.count(mMap.vpKeyFrames[i]) == 0)
            continue;

        int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[i]];
        for(meas_it it= mMap.vpKeyFrames[i]->mMeasurements.begin();
            it!= mMap.vpKeyFrames[i]->mMeasurements.end();
            it++)
        {

            if(mPoint_BundleID.count(it->first) == 0)
                continue;
            int nPoint_BundleID = mPoint_BundleID[it->first];
            Measurement& m = it->second;
            if (m.dDepth > 0.0) {
                static gvar3<double> gvdDepthErrorScale("Tracker.DepthErrorScale",0.0025,SILENT);
                if (*gvnUseDepth) {
                    double dStdDepth = m.dDepth*m.dDepth*(*gvdDepthErrorScale);
                    b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, LevelScale(m.nLevel)*LevelScale(m.nLevel));
                    b.AddMeas(nKF_BundleID, nPoint_BundleID, m.dDepth, dStdDepth*dStdDepth);
                } else if (*gvnUse3D) {
                    double dStdDepth = m.dDepth*m.dDepth*(*gvdDepthErrorScale);
                    Vector<3> v3RootPos = m.dDepth*unproject(mCamera->UnProject(m.v2RootPos));
                    b.AddMeas(nKF_BundleID, nPoint_BundleID, v3RootPos,dStdDepth*dStdDepth);
                } else {
                    b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, LevelScale(m.nLevel)*LevelScale(m.nLevel));
                }
            } else {
                b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, LevelScale(m.nLevel)*LevelScale(m.nLevel));
            }
        }
    }
    // and the adjusted and fixed kfs from the second camera
    for (int cn = 0; cn < AddCamNumber; cn ++){
        if (!mMap.vpKeyFramessec[cn].size()) continue;
        for(unsigned int i=0; i<mMap.vpKeyFramessec[cn].size(); i++)
        {
            if (mView_BundleID.count(mMap.vpKeyFramessec[cn][i]) == 0)
                continue;

            int nKF_BundleID = mView_BundleID[mMap.vpKeyFramessec[cn][i]];
            for(meas_it it= mMap.vpKeyFramessec[cn][i]->mMeasurements.begin();
                it!= mMap.vpKeyFramessec[cn][i]->mMeasurements.end();
                it++)
            {
                if(mPoint_BundleID.count(it->first) == 0)
                    continue;
                int nPoint_BundleID = mPoint_BundleID[it->first];
                Measurement& m = it->second;
                if (m.dDepth > 0.0) {
                    static gvar3<double> gvdDepthErrorScale("Tracker.DepthErrorScale",0.0025,SILENT);
                    if (*gvnUseDepth) {
                        double dStdDepth = m.dDepth*m.dDepth*(*gvdDepthErrorScale);
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, LevelScale(m.nLevel)*LevelScale(m.nLevel), cn);
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, m.dDepth, dStdDepth*dStdDepth, cn);
                    } else if (*gvnUse3D) {
                        double dStdDepth = m.dDepth*m.dDepth*(*gvdDepthErrorScale);
                        Vector<3> v3RootPos = m.dDepth*unproject(mCameraSec[cn]->UnProject(m.v2RootPos));
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, v3RootPos,dStdDepth*dStdDepth, cn);
                    } else {
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, 4*LevelScale(m.nLevel)*LevelScale(m.nLevel), cn);
                    }
                } else {
                    b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, 4*LevelScale(m.nLevel)*LevelScale(m.nLevel), cn);
                }
            }
        }
    }
    // finally those associated kfs from the second cam
    /// in the current conf., all kfs from addcams should be here
    for (int cn = 0; cn <AddCamNumber; cn ++){
        for(unsigned int i=0; i<mMap.vpKeyFramessec[cn].size(); i++)
        {
            if(!sAssociatedSet.count(mMap.vpKeyFramessec[cn][i]))
                continue;

            // for these kfs, we actually assign their mappoints to the associated kfs from the first cam
            // so when add those measurements, special care should be taken
            // in BA, only they have to be treated differently
            int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[mMap.vpKeyFramessec[cn][i]->nAssociatedKf]];
            for(meas_it it= mMap.vpKeyFramessec[cn][i]->mMeasurements.begin();
                it!= mMap.vpKeyFramessec[cn][i]->mMeasurements.end();
                it++)
            {
                if(mPoint_BundleID.count(it->first) == 0)
                    continue;
                int nPoint_BundleID = mPoint_BundleID[it->first];
                Measurement& m = it->second;
                if (m.dDepth > 0.0) {
                    static gvar3<double> gvdDepthErrorScale("Tracker.DepthErrorScale",0.0025,SILENT);
                    if (*gvnUseDepth) {
                        double dStdDepth = m.dDepth*m.dDepth*(*gvdDepthErrorScale);
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, LevelScale(m.nLevel)*LevelScale(m.nLevel), cn);
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, m.dDepth, dStdDepth*dStdDepth, cn);
                    } else if (*gvnUse3D) {
                        double dStdDepth = m.dDepth*m.dDepth*(*gvdDepthErrorScale);
                        Vector<3> v3RootPos = m.dDepth*unproject(mCameraSec[cn]->UnProject(m.v2RootPos));
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, v3RootPos,dStdDepth*dStdDepth, cn);
                    } else {// do main change to such measurement
                        b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, 4*LevelScale(m.nLevel)*LevelScale(m.nLevel), cn, true);
                    }
                } else {
                    // do main change to such measurement
                    b.AddMeas(nKF_BundleID, nPoint_BundleID, m.v2RootPos, 4*LevelScale(m.nLevel)*LevelScale(m.nLevel), cn, true);
                }
            }
        }
    }
    cout << "Doing LBA Compute..." << endl;

    // Run the bundle adjuster. This returns the number of successful iterations
    int nAccepted = b.Compute(&mbBundleAbortRequested);
    cout << "LBA done, updating the map..." << endl;

    if(nAccepted < 0)
    {
        // Crap: - LM Ran into a serious problem!
        // This is probably because the initial stereo was messed up.
        // Get rid of this map and start again!
        cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
             << "   The map is probably corrupt: Ditching the map. " << endl;
        mbResetRequested = true;
        return;
    }

    // Bundle adjustment did some updates, apply these to the map
    if(nAccepted > 0)
    {
        // yangs: here a mutex lock should be added for map manipulate.
        // write access: unique lock
        boost::unique_lock< boost::shared_mutex > lock(mMap.mutex);

        for(map<boost::shared_ptr<MapPoint>, int>::iterator itr = mPoint_BundleID.begin();
            itr!=mPoint_BundleID.end();
            itr++){
            itr->first->v3WorldPos = b.GetPoint(itr->second);
            //TODO: and update the relative pose for full slam, careful about its source kf, which may be either fixed or adjusted.
//            itr->first->v3SourceKFfromeWorld =
//                    pNew->v3SourceKFfromeWorld = kSrc->se3CfromW;
//                    pNew->v3RelativePos = kSrc->se3CfromW * pNew->v3WorldPos;

        }
//        cout << "Map points updated." << endl;

        for(map<boost::shared_ptr<KeyFrame>,int>::iterator itr = mView_BundleID.begin();
            itr!=mView_BundleID.end();
            itr++){
            itr->first->se3CfromW = b.GetCamera(itr->second);
            // Also update their association
            if (*gvnSecondCamIndependent == 0){
//                if (itr->first->nSourceCamera && itr->first->mAssociateKeyframe){// kfs from the second cam
//                    mMap.vpKeyFrames[itr->first->nAssociatedKf]->se3CfromW = itr->first->se3Cam2fromCam1.inverse()*itr->first->se3CfromW;
//                }
//                else

                /// TODO: take care of the case when itr->first->nAssociatedKf may point to different kfs from the multi-cam.
                /// i.e. the sizes of kfs from multi-cam. may be different
                for (int cn = 0; cn < AddCamNumber; cn ++)
                    if (!itr->first->nSourceCamera && itr->first->mAssociateKeyframe
                        && sAssociatedSet.count(mMap.vpKeyFramessec[cn][itr->first->nAssociatedKf]))
                    mMap.vpKeyFramessec[cn][itr->first->nAssociatedKf]->se3CfromW = itr->first->se3Cam2fromCam1*itr->first->se3CfromW;
            }
        }
        if(bRecent)
            mbBundleConverged_Recent = false;
        mbBundleConverged_Full = false;
//        cout << "Keyframe poses updated." << endl;

        lock.unlock();
    };

    if(b.Converged())
    {
        mbBundleConverged_Recent = true;
        if(!bRecent)
            mbBundleConverged_Full = true;
    }

    mbBundleRunning = false;
    mbBundleAbortRequested = false;

    static gvar3<int> gvnAlwaysHandleOutliers("MapMaker.AlwaysHandleOutliers", 0, SILENT);
    if (!bRecent || *gvnAlwaysHandleOutliers == 1) {
        // Handle outlier measurements:
        vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
        unsigned int nBadPoints = 0;
        for(unsigned int i=0; i<vOutliers_PC_pair.size(); i++)
        {
            boost::shared_ptr<MapPoint> pp = mBundleID_Point[vOutliers_PC_pair[i].first];
            boost::shared_ptr<KeyFrame> pk = mBundleID_View[vOutliers_PC_pair[i].second];
            Measurement &m = pk->mMeasurements[pp];
            if(pp->MMData.GoodMeasCount() < 2 || m.Source == Measurement::SRC_ROOT) {  // Is the original source kf considered an outlier? That's bad.
                pp->bBad = true;
                nBadPoints++;
            } else {
                // Do we retry it? Depends where it came from!!
                if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
                    mvFailureQueue.push_back(pair<boost::shared_ptr<KeyFrame>, boost::shared_ptr<MapPoint> >(pk,pp));
                else
                    pp->MMData.sNeverRetryKFs.insert(pk);
                pk->mMeasurements.erase(pp);
                pp->MMData.sMeasurementKFs.erase(pk);
            }
        }
        //  std::cout << __PRETTY_FUNCTION__ << " bad map points " << nBadPoints << std::endl;
    }
}

// update the existed keyframes and related map points in the last BA step
// method 1: update each kf and its managed points
// method 2: assume the local map to be a rigid body with only possiblly scale change in pgo, then apply global transform to the local map
// the second one should be simpler. How to justify this method?
// and another problem is how to apply a scale correction among those local nodes?
void MapMaker::UpdateLMapByGMap(){
    cout << "Updating Local map by GMAP..." << "\n";
    std::vector<SE3<> > kfspose;

    // label each map point not updated
    for(unsigned int j = 0; j < mMap.vpPoints.size(); j ++){
        boost::shared_ptr<MapPoint> vpPoint = mMap.vpPoints[j];
        vpPoint->bUpdated = false;
    }

    // only keyframes from the second cam K2S is used in the backend and can be updated by GMAP,
    // associated kfs from master cam are updated by corresponding K2S
    for(unsigned int i = 0; i < mMap.vpKeyFrames.size(); i ++){
        kfspose.push_back(mMap.vpKeyFrames[i]->se3CfromW);

        mMap.vpKeyFrames[i]->updatedByGMap = false;
        if (mMap.vpKeyFrames[i]->id > (mSLAM.keyframes_.size()-1)){// note new kfs may have been added
            // update such kfs according to relative pose to older kf, which has been updated
            // TODO: correct the scale of such kfs in SIM3 optimisation
            for ( int j = i-1; j >=0; j --){
                if (mMap.vpKeyFrames[j]->updatedByGMap){
                    // use this kf to update kf i
                    SE3<> relpose = mMap.vpKeyFrames[i]->se3CfromW * kfspose[j].inverse();
                    mMap.vpKeyFrames[i]->se3CfromW = relpose * mMap.vpKeyFrames[j]->se3CfromW;

                    mMap.vpKeyFrames[i]->updatedByGMap = true;
//                    cout << mMap.vpKeyFrames[i]->se3CfromW << endl;
                    break;
                }
            }
        }
        else{ /// most likely
            // make sure id matched
            assert(mMap.vpKeyFrames[i]->id == mSLAM.keyframes_[mMap.vpKeyFrames[i]->id]->id);
            // to toon se3
//            SE3<> se3wfc = toToonSE3(mSLAM.keyframes_[mMap.vpKeyFrames[i]->id]->posewTc);
            SE3<> se3cfw = cs_geom::toToonSE3(mSLAM.keyframeUpdatedPoses[mMap.vpKeyFrames[i]->id]);
            mMap.vpKeyFrames[i]->se3CfromW = se3cfw;
            mMap.vpKeyFrames[i]->updatedByGMap = true;
//            cout << mMap.vpKeyFramessec[i]->se3CfromW << endl;
        }

//        cout << "Keyframe pose updated by GMAP." << "\n";

        // update the map point pose according to relative pose to their source kf poses
        // update: it doesn't make sense to use the removen kf id, and a better way can be:
        // chose the still existing oldest kf which also measure this point! We update the map point data association each time a kf is removed
        for(unsigned int j = 0; j < mMap.vpPoints.size(); j ++){
            // TODO: if RBA, update the relative pose for full slam, careful about its source kf, which may be either fixed or adjusted.
            // TODO: update the scale when pgo in SIM3
            // TODO: to be more efficient: check whether a point has been updated already
            boost::shared_ptr<MapPoint> vpPoint = mMap.vpPoints[j];
            if(!vpPoint->bUpdated && (vpPoint->nSourceCamera==0) && (mMap.vpKeyFrames[i] == vpPoint->pPatchSourceKF.lock())){
                Vector<3> rpos = kfspose[i] * vpPoint->v3WorldPos;
                vpPoint->v3WorldPos = mMap.vpKeyFrames[i]->se3CfromW.inverse() * rpos;
            }
        }
//        cout << "Keyframe points updated by GMAP." << "\n";

        // Also update their associated kfs. other first cam kfs without association from the second cam (now cannot happen), should be updated according to their neighbores!
        /// TODO: whether we want to associate additional kfs with first cam together or individually?
        if (mMap.vpKeyFrames[i]->mAssociateKeyframe){// kfs from the second cam
            for (int cn = 0; cn < AddCamNumber; cn ++){
                if (mMap.vpKeyFramessec[cn].size() <= mMap.vpKeyFrames[i]->nAssociatedKf)
                    continue;
                SE3<> kfpos = mMap.vpKeyFramessec[cn][mMap.vpKeyFrames[i]->nAssociatedKf]->se3CfromW;
                mMap.vpKeyFramessec[cn][mMap.vpKeyFrames[i]->nAssociatedKf]->se3CfromW = mse3Cam2FromCam1[cn]*mMap.vpKeyFrames[i]->se3CfromW;
                //            cout << mMap.vpKeyFrames[i]->se3CfromW << endl;
                //            assert(mMap.vpKeyFramessec[i]->nAssociatedKf == i);
                // and update its map points
                for(unsigned int j = 0; j < mMap.vpPoints.size(); j ++){
                    // TODO: if RBA, update the relative pose for full slam, careful about its source kf, which may be either fixed or adjusted.
                    // TODO: update the scale if pgo in SIM3
                    boost::shared_ptr<MapPoint> vpPoint = mMap.vpPoints[j];
                    if((vpPoint->nSourceCamera == (cn + 1)) && (mMap.vpKeyFramessec[cn][mMap.vpKeyFrames[i]->nAssociatedKf] == vpPoint->pPatchSourceKF.lock())){
                        Vector<3> rpos = kfpos * vpPoint->v3WorldPos;
                        vpPoint->v3WorldPos = mMap.vpKeyFramessec[cn][mMap.vpKeyFrames[i]->nAssociatedKf]->se3CfromW.inverse() * rpos;
                    }
                }
            }
        }
//        cout << "Keyframe associated updated by GMAP." << "\n";
    }
    cout << "Updated Local map by GMAP..." << "\n";
}

// the update of map points is naturally done, since in the backend they are represented in a relative manner
void MapMaker::UpdateWaitingList(){
    for (unsigned int i = 0; i < mSLAM.wlKeyFrames.size(); i ++){
        for (unsigned int j = 0; j < mMap.vpKeyFramessec[0].size(); j ++){
            if (mMap.vpKeyFramessec[0][j]->id == mSLAM.wlKeyFrames[i]){//->id
//                mSLAM.wlKeyFrames[i]->ptamPosewTc = cs_geom::toSophusSE3(mMap.vpKeyFramessec[j]->se3CfromW);
//                mSLAM.wlKeyFrames[i]->posewTc = mSLAM.wlKeyFrames[i]->ptamPosewTc;
            }
        }
    }
    // and updates those related edges, maybe not needed since they will be overwriten later
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in 
// TrackerData.h.
bool MapMaker::ReFind_Common(boost::shared_ptr<KeyFrame> k, boost::shared_ptr<MapPoint> p)
{
    int nCam= k->nSourceCamera;
    // abort if either a measurement is already in the map, or we've
    // decided that this point-kf combo is beyond redemption
    if(p->MMData.sMeasurementKFs.count(k)
            || p->MMData.sNeverRetryKFs.count(k))
        return false;

    // this line was missing in original ptam code. if you remove it, the assertion at the bottom
    // will fail from time to time
    if (k->mMeasurements.count(p))
        return false;

    static PatchFinder Finder;
    Vector<3> v3Cam = k->se3CfromW*p->v3WorldPos;
    if(v3Cam[2] < 0.001)
    {
        p->MMData.sNeverRetryKFs.insert(k);
        return false;
    }
    Vector<2> v2ImPlane = project(v3Cam);
    if (nCam){
        if( v2ImPlane* v2ImPlane > mCameraSec[nCam - 1]->LargestRadiusInImage() * mCameraSec[nCam - 1]->LargestRadiusInImage())
        {
            p->MMData.sNeverRetryKFs.insert(k);
            return false;
        }
    }
    else if(v2ImPlane* v2ImPlane > mCamera->LargestRadiusInImage() * mCamera->LargestRadiusInImage())
    {
        p->MMData.sNeverRetryKFs.insert(k);
        return false;
    }

    Vector<2> v2Image;
    if (nCam)
    {
        v2Image = mCameraSec[nCam - 1]->Project(v2ImPlane);
        if(mCameraSec[nCam - 1]->Invalid())
        {
            p->MMData.sNeverRetryKFs.insert(k);
            return false;
        }
    }
    else
    {
        v2Image = mCamera->Project(v2ImPlane);
        if(mCamera->Invalid())
        {
            p->MMData.sNeverRetryKFs.insert(k);
            return false;
        }
    }

    ImageRef irImageSize = k->aLevels[0].im.size();
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
    {
        p->MMData.sNeverRetryKFs.insert(k);
        return false;
    }

    Matrix<2> m2CamDerivs;
    if (nCam)
        m2CamDerivs = mCameraSec[nCam - 1]->GetProjectionDerivs();
    else
        m2CamDerivs = mCamera->GetProjectionDerivs();
    Finder.MakeTemplateCoarse(*p, k->se3CfromW, m2CamDerivs);

    if(Finder.TemplateBad())
    {
        p->MMData.sNeverRetryKFs.insert(k);
        return false;
    }

    bool bFound = Finder.FindPatchCoarse(ir(v2Image), *k, 4);  // Very tight search radius!
    if(!bFound)
    {
        p->MMData.sNeverRetryKFs.insert(k);
        return false;
    }

    // If we found something, generate a measurement struct and put it in the map
    Measurement m;
    m.nLevel = Finder.GetLevel();
    m.dDepth = Finder.GetCoarseDepth();
    m.Source = Measurement::SRC_REFIND;

    if(Finder.GetLevel() > 0)
    {
        Finder.MakeSubPixTemplate();
        Finder.IterateSubPixToConvergence(*k,8);
        m.v2RootPos = Finder.GetSubPixPos();
        m.bSubPix = true;
    }
    else
    {
        m.v2RootPos = Finder.GetCoarsePosAsVector();
        m.bSubPix = false;
    };

    if(k->mMeasurements.count(p))
    {
        assert(0); // This should never happen, we checked for this at the start.
    }
    k->mMeasurements[p] = m;
    p->MMData.sMeasurementKFs.insert(k);
    return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(boost::shared_ptr<KeyFrame> k)
{
    int nCam = k->nSourceCamera;
    vector<boost::shared_ptr<MapPoint> > vToFind;
    for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
        // we treat map points from dual camera separately
        if (mMap.vpPoints[i]->nSourceCamera == nCam)
            vToFind.push_back(mMap.vpPoints[i]);
    }

    int nFoundNow = 0;
    for(unsigned int i=0; i<vToFind.size(); i++)
        if(ReFind_Common(k, vToFind[i]))
            nFoundNow++;

    return nFoundNow;
};

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
    if(mqNewQueue.empty())
        return;
    int nFound = 0;
    int nBad = 0;
    bool nullkfsecs = true;
    for (int i = 0; i < AddCamNumber; i ++)
        if (mvpKeyFrameQueueSec[i].size())
            nullkfsecs = false;
    while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0
          && nullkfsecs)
    {
        boost::shared_ptr<MapPoint> pNew = mqNewQueue.front();
        mqNewQueue.pop();
        if(pNew->bBad)
        {
            nBad++;
            continue;
        }
        if (!pNew->nSourceCamera){
            for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
                if(ReFind_Common(mMap.vpKeyFrames[i], pNew))
                    nFound++;
        }
        else { // the second camera
            for(unsigned int i=0; i<mMap.vpKeyFramessec[pNew->nSourceCamera - 1].size(); i++)
                if(ReFind_Common(mMap.vpKeyFramessec[pNew->nSourceCamera - 1][i], pNew))
                    nFound++;
        }
    }
};

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
    if(mvFailureQueue.size() == 0)
        return;
    sort(mvFailureQueue.begin(), mvFailureQueue.end());
    vector<pair<boost::shared_ptr<KeyFrame>, boost::shared_ptr<MapPoint> > >::iterator it;
    int nFound=0;
    for(it = mvFailureQueue.begin(); it!=mvFailureQueue.end(); it++)
        if(ReFind_Common(it->first, it->second))
            nFound++;
    mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(boost::shared_ptr<KeyFrame> kCurrent)
{
    return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

void MapMaker::EnableMapping()
{
    boost::mutex::scoped_lock lock(MappingEnabledMut);
    if (!mbMappingEnabled) {
        mbMappingEnabled = true;
        MappingEnabledCond.notify_one();
    }
}

void MapMaker::DisableMapping()
{
    boost::mutex::scoped_lock lock(MappingEnabledMut);
    mbMappingEnabled = false;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMaker::CalcPlaneAligner()
{
    unsigned int nPoints = mMap.vpPoints.size();
    if(nPoints < 10)
    {
        cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
        return SE3<>();
    };

    int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
    Vector<3> v3BestMean;
    Vector<3> v3BestNormal;
    double dBestDistSquared = 9999999999999999.9;

    for(int i=0; i<nRansacs; i++)
    {
        int nA = rand()%nPoints;
        int nB = nA;
        int nC = nA;
        while(nB == nA)
            nB = rand()%nPoints;
        while(nC == nA || nC==nB)
            nC = rand()%nPoints;

        Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos +
                                         mMap.vpPoints[nB]->v3WorldPos +
                                         mMap.vpPoints[nC]->v3WorldPos);

        Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
        Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
        Vector<3> v3Normal = v3CA ^ v3BA;
        if(v3Normal * v3Normal  == 0)
            continue;
        normalize(v3Normal);

        double dSumError = 0.0;
        for(unsigned int i=0; i<nPoints; i++)
        {
            Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
            double dDistSq = v3Diff * v3Diff;
            if(dDistSq == 0.0)
                continue;
            double dNormDist = fabs(v3Diff * v3Normal);

            if(dNormDist > 0.05)
                dNormDist = 0.05;
            dSumError += dNormDist;
        }
        if(dSumError < dBestDistSquared)
        {
            dBestDistSquared = dSumError;
            v3BestMean = v3Mean;
            v3BestNormal = v3Normal;
        }
    }

    // Done the ransacs, now collect the supposed inlier set
    vector<Vector<3> > vv3Inliers;
    for(unsigned int i=0; i<nPoints; i++)
    {
        Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
        double dDistSq = v3Diff * v3Diff;
        if(dDistSq == 0.0)
            continue;
        double dNormDist = fabs(v3Diff * v3BestNormal);
        if(dNormDist < 0.05)
            vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
    }

    // With these inliers, calculate mean and cov
    Vector<3> v3MeanOfInliers = Zeros;
    for(unsigned int i=0; i<vv3Inliers.size(); i++)
        v3MeanOfInliers+=vv3Inliers[i];
    v3MeanOfInliers *= (1.0 / vv3Inliers.size());

    Matrix<3> m3Cov = Zeros;
    for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
        Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
        m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };

    // Find the principal component with the minimal variance: this is the plane normal
    SymEigen<3> sym(m3Cov);
    Vector<3> v3Normal = sym.get_evectors()[0];

    // Use the version of the normal which points towards the cam center
    if(v3Normal[2] > 0)
        v3Normal *= -1.0;

    Matrix<3> m3Rot = Identity;
    m3Rot[2] = v3Normal;
    m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
    normalize(m3Rot[0]);
    m3Rot[1] = m3Rot[2] ^ m3Rot[0];

    SE3<> se3Aligner;
    se3Aligner.get_rotation() = m3Rot;
    Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
    se3Aligner.get_translation() = -v3RMean;

    return se3Aligner;
}

// @param  bestnormal serves as initial model, also as output model
int MapMaker::CheckInlier_Refine(std::vector<boost::shared_ptr<MapPoint> > mpadmappoints,
                                 TooN::SE3<> &landingpadfromworld, Vector<3> &bestnormal,
                                 Vector<3> &bestmean)
{
    unsigned int nPoints = mpadmappoints.size();

    int nRansacs = GV2.GetInt("MapMaker.LandingpadPlaneRansacRefine", 20, HIDDEN|SILENT);
    Vector<3> v3BestMean;
    Vector<3> v3BestNormal;
    double dBestDistSquared = 9999999999999999.9;

    for(int i=0; i<nRansacs; i++)
    {
        Vector<3> v3Normal;
        Vector<3> v3Mean;
        if (i == 0){
            v3Normal = bestnormal;
            if(v3Normal * v3Normal  == 0)
                continue;
            normalize(v3Normal);
            //TODO: also input v3mean!
            v3Mean = bestmean;
        }
        else {
            int nA = rand()%nPoints;
            int nB = nA;
            int nC = nA;
            while(nB == nA)
                nB = rand()%nPoints;
            while(nC == nA || nC==nB)
                nC = rand()%nPoints;

            v3Mean = 0.33333333 * (mpadmappoints[nA]->v3WorldPos +
                                   mpadmappoints[nB]->v3WorldPos +
                                   mpadmappoints[nC]->v3WorldPos);

            Vector<3> v3CA = mpadmappoints[nC]->v3WorldPos  - mpadmappoints[nA]->v3WorldPos;
            Vector<3> v3BA = mpadmappoints[nB]->v3WorldPos  - mpadmappoints[nA]->v3WorldPos;
            v3Normal = v3CA ^ v3BA;
            if(v3Normal * v3Normal  == 0)
                continue;
            normalize(v3Normal);
        }

        double dSumError = 0.0;
        for(unsigned int i=0; i<nPoints; i++)
        {
            Vector<3> v3Diff = mpadmappoints[i]->v3WorldPos - v3Mean;
            double dDistSq = v3Diff * v3Diff;
            if(dDistSq == 0.0)
                continue;
            double dNormDist = fabs(v3Diff * v3Normal);

            if(dNormDist > 0.05)
                dNormDist = 0.05;
            dSumError += dNormDist;
        }
        if(dSumError < dBestDistSquared)
        {
            dBestDistSquared = dSumError;
            v3BestMean = v3Mean;
            v3BestNormal = v3Normal;
        }
    }

    // Done the ransacs, now collect the supposed inlier set
    vector<Vector<3> > vv3Inliers;
    for(unsigned int i=0; i<nPoints; i++)
    {
        Vector<3> v3Diff = mpadmappoints[i]->v3WorldPos - v3BestMean;
        double dDistSq = v3Diff * v3Diff;
        if(dDistSq == 0.0)
            continue;
        double dNormDist = fabs(v3Diff * v3BestNormal);
        if(dNormDist < 0.05)
            vv3Inliers.push_back(mpadmappoints[i]->v3WorldPos);
    }

    // With these inliers, calculate mean and cov
    Vector<3> v3MeanOfInliers = Zeros;
    for(unsigned int i=0; i<vv3Inliers.size(); i++)
        v3MeanOfInliers+=vv3Inliers[i];
    v3MeanOfInliers *= (1.0 / vv3Inliers.size());

    Matrix<3> m3Cov = Zeros;
    for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
        Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
        m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };

    // Find the principal component with the minimal variance: this is the plane normal
    SymEigen<3> sym(m3Cov);
    Vector<3> v3Normal = sym.get_evectors()[0];

    // Use the version of the normal which points towards upward
    if(v3Normal[2] < 0)
        v3Normal *= -1.0;

    // check whether the estimation is sound, simply by checking the normal vector
    // angle of the normal vector to z axis of world frame should be less than Pi/6
    Vector<3> padnormal = v3Normal;
    normalize(padnormal);
    if (acos(padnormal[2]) > 3.14/6.0)
        return false;

    Matrix<3> m3Rot = Identity;
    m3Rot[2] = v3Normal;
    m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
    normalize(m3Rot[0]);
    m3Rot[1] = m3Rot[2] ^ m3Rot[0];

    SE3<> se3Aligner;
    se3Aligner.get_rotation() = m3Rot;
    Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
    se3Aligner.get_translation() = -v3RMean;


    landingpadfromworld = se3Aligner;
    bestnormal = v3Normal;
    bestmean = v3MeanOfInliers;//se3Aligner.get_translation();
    return true;
}

bool MapMaker::CalcLandingpadPlane(std::vector<boost::shared_ptr<MapPoint> > mpadmappoints,
                                   TooN::SE3<> &landingpadfromworld, Vector<3> &bestnormal, Vector<3> &bestmean)
{
    unsigned int nPoints = mpadmappoints.size();
    //  if(nPoints < 10)
    //    {
    //      cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
    //      return SE3<>();
    //    };

    int nRansacs = GV2.GetInt("MapMaker.LandingpadPlaneRansacs", 30, HIDDEN|SILENT);
    Vector<3> v3BestMean;
    Vector<3> v3BestNormal;
    //TODO: also record the second best one, this is useful for special scenarioes like landing on a box or table
    double dBestDistSquared = 9999999999999999.9;

    for(int i=0; i<nRansacs; i++)
    {
        int nA = rand()%nPoints;
        int nB = nA;
        int nC = nA;
        while(nB == nA)
            nB = rand()%nPoints;
        while(nC == nA || nC==nB)
            nC = rand()%nPoints;

        Vector<3> v3Mean = 0.33333333 * (mpadmappoints[nA]->v3WorldPos +
                                         mpadmappoints[nB]->v3WorldPos +
                                         mpadmappoints[nC]->v3WorldPos);

        Vector<3> v3CA = mpadmappoints[nC]->v3WorldPos  - mpadmappoints[nA]->v3WorldPos;
        Vector<3> v3BA = mpadmappoints[nB]->v3WorldPos  - mpadmappoints[nA]->v3WorldPos;
        Vector<3> v3Normal = v3CA ^ v3BA;
        if(v3Normal * v3Normal  == 0)
            continue;
        normalize(v3Normal);

        double dSumError = 0.0;
        for(unsigned int i=0; i<nPoints; i++)
        {
            Vector<3> v3Diff = mpadmappoints[i]->v3WorldPos - v3Mean;
            double dDistSq = v3Diff * v3Diff;
            if(dDistSq == 0.0)
                continue;
            double dNormDist = fabs(v3Diff * v3Normal);

            if(dNormDist > 0.05)
                dNormDist = 0.05;
            dSumError += dNormDist;
        }
        if(dSumError < dBestDistSquared)
        {
            dBestDistSquared = dSumError;
            v3BestMean = v3Mean;
            v3BestNormal = v3Normal;
        }
    }

    // Done the ransacs, now collect the supposed inlier set
    vector<Vector<3> > vv3Inliers;
    for(unsigned int i=0; i<nPoints; i++)
    {
        Vector<3> v3Diff = mpadmappoints[i]->v3WorldPos - v3BestMean;
        double dDistSq = v3Diff * v3Diff;
        if(dDistSq == 0.0)
            continue;
        double dNormDist = fabs(v3Diff * v3BestNormal);
        if(dNormDist < 0.05)
            vv3Inliers.push_back(mpadmappoints[i]->v3WorldPos);
    }

    // With these inliers, calculate mean and cov
    Vector<3> v3MeanOfInliers = Zeros;
    for(unsigned int i=0; i<vv3Inliers.size(); i++)
        v3MeanOfInliers+=vv3Inliers[i];
    v3MeanOfInliers *= (1.0 / vv3Inliers.size());

    Matrix<3> m3Cov = Zeros;
    for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
        Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
        m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };

    // Find the principal component with the minimal variance: this is the plane normal
    SymEigen<3> sym(m3Cov);
    Vector<3> v3Normal = sym.get_evectors()[0];

    // Use the version of the normal which points towards upward
    if(v3Normal[2] < 0)
        v3Normal *= -1.0;

    // check whether the estimation is sound, simply by checking the normal vector
    // angle of the normal vector to z axis of world frame should be less than Pi/6
    Vector<3> padnormal = v3Normal;
    normalize(padnormal);
    if (acos(padnormal[2]) > 3.14/6.0)
        return false;

    Matrix<3> m3Rot = Identity;
    m3Rot[2] = v3Normal;
    m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
    normalize(m3Rot[0]);
    m3Rot[1] = m3Rot[2] ^ m3Rot[0];

    SE3<> se3Aligner;
    se3Aligner.get_rotation() = m3Rot;
    Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
    se3Aligner.get_translation() = -v3RMean;// this is T21, 1:world frame, 2: pad plane


    landingpadfromworld = se3Aligner;
    bestnormal = v3Normal;
    bestmean = v3MeanOfInliers;//se3Aligner.get_translation();
    return true;

}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(boost::shared_ptr<KeyFrame> pKF)
{
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
        MapPoint &point = *it->first;
        Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
        dSumDepth += v3PosK[2];
        dSumDepthSquared += v3PosK[2] * v3PosK[2];
        nMeas++;
    }

    assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
    pKF->dSceneDepthMean = dSumDepth / nMeas;
    pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
    Command c;
    c.sCommand = sCommand;
    c.sParams = sParams;
    ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
    if(sCommand=="SaveMap")
    {
        cout << "  MapMaker: Saving the map.... " << endl;
        ofstream ofs("map.dump");
        for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
        {
            ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
            ofs << mMap.vpPoints[i]->nSourceLevel << endl;
        }
        ofs.close();

        for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        {
            ostringstream ost1;
            ost1 << "keyframes/" << i << ".jpg";
            //	  img_save(mMap.vpKeyFrames[i]->aLevels[0].im, ost1.str());

            ostringstream ost2;
            ost2 << "keyframes/" << i << ".info";
            ofstream ofs2;
            ofs2.open(ost2.str().c_str());
            ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
            ofs2.close();
        }
        cout << "  ... done saving map." << endl;
        return;
    }

    cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
    exit(1);
}

void MapMaker::WriteFrames(const char* fname)
{
    ofstream ofs("frames.csv");
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
        ofs << mMap.vpKeyFrames[i]->se3CfromW << endl;
    }
    ofs.close();
}










