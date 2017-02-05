//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "CameraModel.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include <sstream>
#include <vector>
#include <list>
#include <memory>
#include <sensor_msgs/PointCloud.h>
#include <boost/smart_ptr.hpp>
#include <TooN/wls.h>
#include <TooN/SVD.h>
#include <iostream>
#include <fstream>
#include <string>

namespace ptam{
class MapPoint;

struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

class Tracker
{
public:
  Tracker(Map &m, MapMaker &mm);
  
  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(CVD::Image<CVD::byte> &imFrame);
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, CVD::Image<CVD::byte> &imFramesec);
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, CVD::Image<uint16_t> &imFrameD);
  void TrackFrame(CVD::Image<CVD::Rgb<CVD::byte> > &imFrameRGB, CVD::Image<uint16_t> &imFrameD, bool isBgr = false);
  ///
  /// \brief TrackFrame
  /// \param imFrameRGB     a list of images from multiple cameras, maybe incomplete
  /// \param imFrameD       the depth image
  /// \param adcamIndex     handle incomplete cases, record the camera indexes of input images from the additional cameras
  /// \param isBgr
  ///
  void TrackFrame(std::vector<CVD::Image<CVD::Rgb<CVD::byte> > > &imFrameRGB, std::vector<CVD::Image<uint16_t> > &imFrameD, std::vector<int> adcamIndex, bool isBgr = false);
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, const sensor_msgs::PointCloud& points);

  const CameraModel& GetCamera() const { return *mCamera; }
  const CameraModel& GetCameraSec(int addcamnum) const { return *mCameraSec[addcamnum]; }
  int GetNFrame() const { return mnFrame; }
  int GetNLastKeyFrame() const { return mnLastKeyFrameDropped; }
  int GetNKeyFrames() const { return mnKeyFrames; }

  inline SE3<> GetCurrentPose() const { return mse3CamFromWorldPub;}
  inline SE3<> GetCurrentPosesec(int adcamIndex) const { return mse3CamFromWorldsec[adcamIndex];}
  const KeyFrame& GetCurrentKeyFrame() const { return *mCurrentKF; }
  const KeyFrame& GetCurrentsecKeyFrame(int adcamIndex = 0) const { return *mCurrentKFsec[adcamIndex]; }
  Matrix<6, 6> GetPoseCovariance() const;

  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
  
  // initialise the map using circle pattern
  SE3<> se3CfromW;
  SE3<> se3IMU_camfromW;//use attitude info from IMU, and height info of the quadrotor. already been transformed as se3CamfromWorld
  SE3<> se3CfromW_predicted_ekf;
  double scale_ekf;// the scale factor of the pose estimation in the previous image frame, estimated by EKF, which does not change between two image frames

  double time_stamp_circle;
  double time_stamp_now;
  bool use_kinect;
  bool use_circle_ini;
  bool use_one_circle;
  bool use_two_circle;
  bool use_ground;
  bool circle_pose_get;
  bool attitude_get;
  bool ini_circle_finished;
  bool pose_ekf_get;
  bool isflying;
  std::ofstream trackerlog;
  std::ofstream trackerlogdebug;

  bool tracking_map;

  //////////// for landing pad detection////////////
  void load_reflandingpad(std::string &path);
  const KeyFrame& GetRefFrame() const { return mReflandingpadFrame; }
  bool istrackPad;
  TooN::Vector<3> mPadCenterWorld;// world coordinate of the landing pad center
  std::vector<TooN::Vector<3> > mPadCornersWorld;// mainly used for visualization
  TooN::Vector<3> iniPadCenterWorld;
  bool isLandingpadPoseGet; // if true, the pose of landing pad could be published
  bool isFinishPadDetection;// after a period of detection and pad pose obtained, stop detection
  TooN::Vector<3> iniPadCameraPoseWorld;// camera pose when the landing pad is first detected
  TooN::Vector<3> finishPadCameraPoseWorld;// camera pose when the landing pad is last detected

  ///////////// using multiple imgs ////////////
  bool mUsingDualImg; /// abusing "dual" in this project, which means multiple now
  bool mUseDualshould;
  void Load_Cam1FromCam2 (const std::vector<SE3<> > cam1fromcam2){
      assert(cam1fromcam2.size() != AddCamNumber);
      for (int i = 0; i < AddCamNumber; i ++){
          mse3Cam1FromCam2[i] = cam1fromcam2[i];
          mMapMaker.Load_Cam2FromCam1(cam1fromcam2[i].inverse(), i);
      }
      mUseDualshould = true;
  }

  void MotionModelUpdateByGMap(); // update the motion model if the local map is updated with the global map
  bool debugmarkLoopDetected;

  double timecost_vo;// time cost of the VO in each image frame.

protected:
  boost::shared_ptr<KeyFrame> mCurrentKF;            // The current working frame as a keyframe struct
  boost::shared_ptr<KeyFrame> mCurrentKFsec[AddCamNumber];            // The current working frame as a keyframe struct
                                                            /// abusing "second" in this project, which means all those additional cameras
  std::vector<int> ActiveAdCamIndex; /// active cameras used in this frame
  boost::shared_ptr<KeyFrame> mGoodKFtoTrack;       /// the most recent well-tracked kf, used when tracking get lost
  boost::shared_ptr<KeyFrame> mGoodKFtoTracksec[AddCamNumber];
  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  std::auto_ptr<CameraModel> mCamera;             // Projection model
  std::auto_ptr<CameraModel> mCameraSec[AddCamNumber];             // Projection model of the second camera
                                  // transformation w.r.t the master camera need to be counted.
  Relocaliser mRelocaliser;       // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image or (0,0) if we don't know yet.
  
  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  void UpdateImageSize(const CVD::ImageRef& size); // Check whether we need to set image size and do so if necessary.

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
  
  // Methods for tracking the map once it has been made:
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void ApplyMotionModel_EKF();        // Using EKF information
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<boost::shared_ptr<MapPoint> > &vTD, 
		      int nRange, 
              int nFineIts,
              int nCamera = 0);  // Finds points in the image
  Vector<6> CalcPoseUpdate(std::vector<boost::shared_ptr<MapPoint> > vTD, 
			   double dOverrideSigma = 0.0, 
               bool bMarkOutliers = false,
               bool debug = false); // Updates pose from found points.
  Vector<12> CalcPoseUpdateDualCam(std::vector<boost::shared_ptr<MapPoint> > vTD,
               double dOverrideSigma = 0.0,
               bool bMarkOutliers = false,
               bool debug = false); // Updates pose from found points. Also update camera-to-camera calibration error

  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3CamFromWorldsec[AddCamNumber];           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3Cam1FromCam2[AddCamNumber];           // second camera pose in the master camera frames
  Vector<6> mv6cam2fromcam1Error;   // the error of calibrated cam21 transform

  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  Vector<6> mv6CameraVelocity;    // Motion model
  Vector<6> mv6CameraVelocitysec[AddCamNumber];    // Motion model for the second cam
  double mdVelocityMagnitude;     // Used to decide on coarse tracking
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

  // eth
  Matrix<6>	mmCovariances;		// covariance of current converged pose estimate

  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  int mnKeyFrames;                // Number of keyframes inserted.
  // aslo the kfs from the second camera
  int mnFramesec[AddCamNumber];                    // Frames processed since last reset
  int mnLastKeyFrameDroppedsec[AddCamNumber];      // Counter of last keyframe inserted.
  int mnKeyFramessec[AddCamNumber];                // Number of keyframes inserted.

  WLS<6> wls; // Weighted least square solver
  WLS<12> wls2; // Weighted least square solver when included cam2cam calibration error

  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe
  
  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  int mnFailureFrames; // tracking failure frames after tring to recover
  bool mbOldMaplocked; // lock the map when tracking failure occurs, until loop closing
  
  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool AttemptRecovery(boost::shared_ptr<KeyFrame> goodkf, boost::shared_ptr<KeyFrame> kf, TooN::SE3<> &mse3Best, double minInliers = 0.50);
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Vector<6> mv6SBIRot;
  // second image sbi should be calc seperately
  SmallBlurryImage * mpSBILastFramesec[AddCamNumber];
  SmallBlurryImage * mpSBIThisFramesec[AddCamNumber];
  Vector<6> mv6SBIRotSec[AddCamNumber];
  Vector<6> mv6SBIRotDual;// the final estimation from dual sbi
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;
  
  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
  
  // Common tracking methods
  void initNewFrame();
  void initNewFrame_sec( int camIndex = 0);
  bool trackMap();
  bool trackMapDual();
  void processGUIEvents();

  //auto initialization
//  void TrackForInitialMap_OneCircle();      // Use one circle to initialize the map. This is called by TrackFrame if there is not a map yet.

  SE3<> mse3CamFromWorldPub;        // yang, Camera pose for public access.
  void MotionCheck();

  ros::Time time_last_detect;// last ros time, when the last new frame is passed to mapmaker
  KeyFrame  mReflandingpadFrame;// ref image of the landing pad
};
} // namespace

#endif
