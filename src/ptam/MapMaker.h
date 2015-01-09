// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>
#include <gvars3/gvars3.h>

#include "Map.h"
#include "KeyFrame.h"
#include "CameraModel.h"
#include <queue>
#include <memory>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/smart_ptr.hpp>

#include <ODT/landing_object.h>
#include <slam/SLAMSystem.h>
#include <backend.h>

namespace ptam{
//typedef boost::function<void(boost::shared_ptr<KeyFrame>)> sendKfCbFunction;
//typedef boost::function<void(const std::vector<boost::shared_ptr<KeyFrame> >&)> sendEdgesCbFunction;

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread
{
public:
  MapMaker(Map &m, cslam::SLAMSystem &ss, cslam::backend &be, bool bOffline = false);
  ~MapMaker();
  
  // Make a map from scratch. Called by the tracker.
  bool InitFromRGBD(KeyFrame &kf, const TooN::SE3<>& worldPos = TooN::SE3<>());
  bool InitFromRGBD(KeyFrame &kf, KeyFrame* adkfs, const TooN::SE3<>& worldPos = TooN::SE3<>());

  bool InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond, 
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                      TooN::SE3<> &se3CameraPos,
                      bool use_circle_ini, int ini_thresh, int ini_times);

  bool InitFromStereo_OLD(KeyFrame &kFirst, KeyFrame &kSecond,  // EXPERIMENTAL HACK
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                      TooN::SE3<> &se3CameraPos);
  
  
  bool AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
                                   // Returns true if keyframe was added or false if mampaker chose to ignore it
                                   // e.g. because mapping is disabled.
  bool AddKeyFrameDual(KeyFrame &k, KeyFrame &ksec);   // Add two key-frame to the map. Called by the tracker.
                                   // Returns true if keyframe was added or false if mampaker chose to ignore it
                                   // e.g. because mapping is disabled.
  bool AddKeyFrameSec(KeyFrame &ksec);  /// Using multiple cameras, adding additional kfs
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(boost::shared_ptr<KeyFrame> kCurrent, const int ncam=0);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(boost::shared_ptr<KeyFrame> kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)

  void EnableMapping();
  void DisableMapping();
  bool GetMappingEnabled() { return mbMappingEnabled; }

  void OfflineStep(bool bDoGlobal); // Does one iteration of run()'s main loop. Use this for offline (single-threaded) processing

  // Debug methods
  void WriteFrames(const char* fname);

  virtual void StopThread();

  //auto ini
  bool InitFromOneCircle(KeyFrame &kFirst,
                      TooN::SE3<> se3CameraPos,
                      int ini_thresh, int ini_times);
  bool InitFromOneCircleDual(KeyFrame &kFirst, KeyFrame &kss,
                      TooN::SE3<> se3CameraPos,
                      int ini_thresh, int ini_times);

  //////////////////////for landing pad detection//////////////////
  bool AddObjectDetectionFrame(KeyFrame &k); // Pass only the current frame to the frame ready for landing
                                             // object detection.
  bool AddReflandingpadFrame(KeyFrame &k);   // load the landing pad reference frame
  void ComputeObjectDetectionFrame(KeyFrame &k); // do opencv related landing pad detection for current frame pad detection,
                                                 // e.g. descriptors, match features, pose estimation.
  bool AddPadTrackingFrame(KeyFrame &k);  // add every new frame for landing pad tracking
  void TracklandingpadESM(KeyFrame &k);

  bool isObject_detected;      // whether landing object is detected and ready for tracking.
  bool isLandingpadPoseGet; // if true, the pose of landing pad could be published
  bool isLandingPoseGetCurrent;// how about in this current frame?
  bool isFinishPadDetection;// after a period of detection and pad pose obtained, stop detection
  TooN::SE3<> GetLandingpadPose() { return mSe3Landingpadfromworld;}
  TooN::Vector<3> mPadCenterWorld;// world coordinate of the landing pad center
  std::vector<TooN::Vector<3> > mPadCornersWorld;// mainly used for visualization
  TooN::Vector<3> iniPadCenterWorld;// the first detected pose, used to control when to finish detection
  TooN::Vector<3> iniPadCameraPoseWorld;// camera pose when the landing pad is first detected
  TooN::Vector<3> finishPadCameraPoseWorld;// camera pose when the landing pad is last detected
  //////////////////////////////////////////////////////////////////

  void Load_Cam2FromCam1 (const SE3<> cam2fromcam1){
      mse3Cam2FromCam1[0] = cam2fromcam1;
  }

  void registerErasedAllCallback(ErasedAllCbFunction const &cb) { mMap.erasedAllCallback = cb; }
//  void registerErasedKFCallback(sendKfCbFunction const &cb) { sendKfCallback = cb; }
//  void registerBaDoneCallback(sendEdgesCbFunction const &cb) { sendEdgesCallback = cb; }
//  sendKfCbFunction sendKfCallback;
//  sendEdgesCbFunction sendEdgesCallback;

  void sendKfCallback(boost::shared_ptr<const ptam::KeyFrame> kf, bool sendpoints = false);// send kf to the backend
  void sendEdgesCallback(const std::vector<boost::shared_ptr<ptam::KeyFrame> > kfs, const int maxkfsize = 5);// send all edges in BA to backend
//  void sendKfPoints(boost::shared_ptr<const ptam::KeyFrame> kf);// update old kf points
//  void updateKfPoses(const std::vector<boost::shared_ptr<ptam::KeyFrame> > kfs);// send all edges in BA to backend
  void UpdateLMapByGMap();  // update the local map if the gmap is updated by pgo
  void UpdateWaitingList(); // update the waitinglist to reflect the local map update by BA
  bool needMotionModelUpdate;// motion model of the tracker need to be updated if the local map is updated according to the global map of the back end
  SE3<> mse3LatestKFpose;// serve as the reference kf for updating motion model in trackerd
  int  lastKFid;// the id of the reference kf
  bool debugmarkLoopDetected;

  unsigned int imageInputCount;// count the image input for checking stoped or not, read access only in the mapmaker
  unsigned int lastimagecount;
  ros::Time lastimageinput;
protected:
  
  Map &mMap;               // The map // in this full slam system, this will be only the local map handled by ptam
  std::auto_ptr<CameraModel> mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
  std::auto_ptr<CameraModel> mCameraSec[AddCamNumber];             // Projection model of the second camera
  virtual void run();      // The MapMaker thread code lives here

//  Map &mGMap;               // the global map of the slam system, handled by the backend, accessed by ptam.
  cslam::SLAMSystem &mSLAM; // the slam system, which contains the global map
                            // mapmaker will only do read access to it
  cslam::backend &mbackend_;// the backend

  // Functions for starting the map from scratch:
  TooN::SE3<> CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);
  /////////////////////////////
  void ApplyGlobalScaleToMap(double dScale);
  
  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame &k, int nLevel);
  void AddSomeMapPoints(int nLevel, int nCam=0); // for dual camera case, add camera number param.
  bool AddPointDepth(boost::shared_ptr<KeyFrame> kSrc, boost::shared_ptr<KeyFrame> kTarget, int nLevel, int nCandidate, int nCam = 0);
  bool AddPointEpipolar(boost::shared_ptr<KeyFrame> kSrc, boost::shared_ptr<KeyFrame> kTarget, int nLevel, int nCandidate, int nCam = 0);// add cam number param
  // Returns point in ref frame B
  TooN::Vector<3> ReprojectPoint(TooN::SE3<> se3AfromB, const TooN::Vector<2> &v2A, const TooN::Vector<2> &v2B);
  double viewAngleDiffPoint(TooN::Vector<3> vB2A, TooN::Vector<3> vB2B);

  // Bundle adjustment functions:
  void BundleAdjust(std::set<boost::shared_ptr<KeyFrame> > , std::set<boost::shared_ptr<KeyFrame> >, std::set<boost::shared_ptr<MapPoint> >, bool, std::set<boost::shared_ptr<KeyFrame> > sAssociatedSet=std::set<boost::shared_ptr<KeyFrame> >());
  void BundleAdjustAll();
  void BundleAdjustRecent();
  void BundleAdjustAllsec(); // ba for all kfs from two camera.

  // Data association functions:
  int ReFindInSingleKeyFrame(boost::shared_ptr<KeyFrame> k, int nCam = 0);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(boost::shared_ptr<KeyFrame> k, boost::shared_ptr<MapPoint> p, int nCam=0);
  void SubPixelRefineMatches(KeyFrame &k, int nLevel);
  
  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(boost::shared_ptr<KeyFrame> kCurrent);
  double KeyFrameDist(KeyFrame &k1, KeyFrame &k2);
  double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
  double KeyFrameAngularDist(KeyFrame &k1, KeyFrame &k2);
  boost::shared_ptr<KeyFrame> ClosestKeyFrame(boost::shared_ptr<KeyFrame> k, double mindist = 0.0, int nCam=0);// for dual camera case, add cam number param
  std::vector<boost::shared_ptr<KeyFrame> > NClosestKeyFrames(boost::shared_ptr<KeyFrame> k, unsigned int N, int nCam = 0);
  void RefreshSceneDepth(boost::shared_ptr<KeyFrame> pKF);

  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;

  // Member variables:
  std::vector<boost::shared_ptr<KeyFrame> > mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector<std::pair<boost::shared_ptr<KeyFrame>, boost::shared_ptr<MapPoint> > > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<boost::shared_ptr<MapPoint> > mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames
  // second img
  std::vector<boost::shared_ptr<KeyFrame> > mvpKeyFrameQueueSec[AddCamNumber];  // Queue of keyframes from the tracker waiting to be processed

  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                         // This sets the scale of the map
  GVars3::gvar3<double> mgvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth, 
                                        // this controls keyframe separation

  bool mbOffline; // Offline (batch) processing without a separate thread.
  
  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?
  
  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.
  bool mbMappingEnabled;            // Is mapping enabled? Allows us to pause mapping.

  boost::mutex MappingEnabledMut;
  boost::condition_variable MappingEnabledCond;

  bool newRecentKF;
  int nAddedKfNoBA;// number of added kfs without BA. too many such kfs may lead to un-continuas graph of the backend

  // for landing object tracking.
  boost::mutex object_keyframeMut;
  boost::mutex tracking_frameMut;
  landing_object* mLandingPad; // landing pad object
  KeyFrame mObject_keyframe;   // image ready for landing object detection
  bool nullObject_keyframe;    //
  SE3<> mObject_keyframePose;
  KeyFrame mObject_detected;   // store the keyframe where the landing pad was detected.
  KeyFrame mTracking_frame;    // keyframe, pass every frame after detected, for pad tracking
  bool nullTracking_frame;     //

  KeyFrame mReflandingpadFrame;
  cv::Mat mPadHomography;// homography between landing pad and current frame
  TooN::Matrix<3> mpadtrackerhomography;// H^0_1 homography by padtracker, this is the
                                        // homography of the original ref img to the first frame in which landing pad is detected
  TooN::Matrix<3> mHlast2thisframe;// H^{n-1}_n, homography between two successive frames, detected by padtracker(ESM)
  TooN::Matrix<3> mTsubimagelast;// initialised by simple x, y coordinate of the lefttop corner, forming this simple translation
  TooN::Matrix<3> mTsubimagethis;// homography of the made subimage to this frame

  CVD::Image<CVD::byte> mPadsubimg;// subimage of the landing pad been tracked.
  CVD::ImageRef msubimglt;// lefttop of the landing pad subimage
  CVD::ImageRef msubimgsize;// and its size

  std::vector<boost::shared_ptr<KeyFrame> > mPaddetectedKeyframes;// record those keyframes in which landing pad was detected
  // function for calculating the landing pad pose/aligning plane
  bool CalcLandingpadPlane(std::vector<boost::shared_ptr<MapPoint> > mpadmappoints,
                           TooN::SE3<> &landingpadfromworld, TooN::Vector<3> &bestnormal, Vector<3> &bestmean);
  int CheckInlier_Refine(std::vector<boost::shared_ptr<MapPoint> > mpadmappoints,
                         TooN::SE3<> &landingpadfromworld, TooN::Vector<3> &bestnormal, Vector<3> &bestmean);

  TooN::SE3<> mSe3Landingpadfromworld;
  TooN::Vector<3> mLandingpadNormal;
  TooN::Vector<3> mLandingpadMean;// mean value of pose values of those mappoints in the landing pad

  int nPositive;// recorde positive and negative cases
  int nNegative;
  ofstream pos_log_;

  SE3<> mse3Cam2FromCam1[AddCamNumber];
  bool bInputStopped; // no more image input
  bool bFullBAfinished; // full BA refinement to the full map has finished;
  bool bStopForFullBA; // robot stopping to do full ba
};
} // namespace

#endif


















