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

            cout << "Done LBA related staff." << endl;
        }

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

bool MapMaker::relocaliseRegister(const boost::shared_ptr<KeyFrame> goodkf, const boost::shared_ptr<KeyFrame> kf, Sophus::SE3d &result, double minInliers)
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
    for(unsigned int l = 0; l < LEVELS; l++) {
        lcount[l] = 0;
        Level& lev = kf.aLevels[l];
        const int nLevelScale = LevelScale(l);

        for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
        {
            double depth = lev.vMaxCornersDepth[i];
            // only consider corners with valid 3d position estimates
            if ((depth <= 0.0) || (depth > mMaxDepth))
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
            p->v3RelativePos = v3CamPos;// Xc
            p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
            p->v3WorldPos = pkFirst->se3CfromW.inverse() * v3CamPos;

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

    mdWiggleScale = 0.1;

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
    for(unsigned int l = 0; l < LEVELS; l++) {
        lcount[l] = 0;
        Level& lev = kf.aLevels[l];
        const int nLevelScale = LevelScale(l);

        for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
        {
            double depth = lev.vMaxCornersDepth[i];
            // only consider corners with valid 3d position estimates
            if ((depth <= 0.0) || (depth > mMaxDepth))
                continue;

            boost::shared_ptr<MapPoint> p(new MapPoint());

            // Patch source stuff:
            p->nSourceCamera = 0;
            p->pPatchSourceKF = pkFirst;
            p->nSourceLevel = l;
            p->v3Normal_NC = makeVector( 0,0,-1);
            p->irCenter = lev.vMaxCorners[i];
            ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);
            p->v3Center_NC = unproject(mCamera->UnProject(irCenterl0));
            p->v3OneDownFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
            p->v3OneRightFromCenter_NC = unproject(mCamera->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

            Vector<3> v3CamPos = p->v3Center_NC*depth;// Xc
            p->v3RelativePos = v3CamPos;// Xc
            p->v3SourceKFfromeWorld = pkFirst->se3CfromW;
            p->v3WorldPos = pkFirst->se3CfromW.inverse() * v3CamPos;

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
    for (int cn = 0; cn < AddCamNumber; cn ++){
        boost::shared_ptr<KeyFrame> pkSec(new KeyFrame());
        *pkSec = *adkfs[cn];
        pkSec->SBI = adkfs[cn]->SBI;
        pkSec->bFixed = true;
        pkSec->se3CfromW = mse3Cam2FromCam1[cn] * worldPos;

        // Construct map from key points
        PatchFinder finder;
        double dSumDepth = 0.0;
        double dSumDepthSquared = 0.0;
        int nMeas = 0;
        pcount[cn+1] = 0;
        for(unsigned int l = 0; l < LEVELS; l++) {
            lcount[l] = 0;
            Level& lev = adkfs[cn]->aLevels[l];
            const int nLevelScale = LevelScale(l);

            for (unsigned int i = 0; i < lev.vMaxCorners.size(); i++)
            {
                double depth = lev.vMaxCornersDepth[i];
                // only consider corners with valid 3d position estimates
                if ((depth <= 0.0) || (depth > mMaxDepth))
                    continue;

                boost::shared_ptr<MapPoint> p(new MapPoint());

                // Patch source stuff:
                p->nSourceCamera = cn + 1;
                p->pPatchSourceKF = pkSec;
                p->nSourceLevel = l;
                p->v3Normal_NC = makeVector( 0,0,-1);
                p->irCenter = lev.vMaxCorners[i];
                ImageRef irCenterl0 = LevelZeroPosIR(p->irCenter,l);
                p->v3Center_NC = unproject(mCameraSec[cn]->UnProject(irCenterl0));
                p->v3OneDownFromCenter_NC = unproject(mCameraSec[cn]->UnProject(irCenterl0 + ImageRef(0,nLevelScale)));
                p->v3OneRightFromCenter_NC = unproject(mCameraSec[cn]->UnProject(irCenterl0 + ImageRef(nLevelScale,0)));

                Vector<3> v3CamPos = p->v3Center_NC*depth;// Xc
                p->v3RelativePos = v3CamPos;
                p->v3WorldPos = pkSec->se3CfromW.inverse() * v3CamPos; // Xw
                p->v3SourceKFfromeWorld = pkSec->se3CfromW;

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
                //bool bGood = finder.IterateSubPixToConvergence(*pkSec,10);
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
                pkSec->mMeasurements[p] = mFirst;
                p->MMData.sMeasurementKFs.insert(pkSec);
                pcount[cn+1]++;
            }
        }

        pkSec->dSceneDepthMean = dSumDepth / nMeas;
        pkSec->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pkSec->dSceneDepthMean) * (pkSec->dSceneDepthMean));

        RefreshSceneDepth(pkSec);
        pkSec->id = 0;
        secKFid[cn] ++;

        mMap.vpKeyFramessec[cn].push_back(pkSec);
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

/*bool MapMaker::AddObjectDetectionFrame(KeyFrame &k)
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
}*/
