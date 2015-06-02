// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This file declares the MapPoint class
// 
// The map is made up of a bunch of mappoints.
// Each one is just a 3D point in the world;
// it also includes information on where and in which key-frame the point was
// originally made from, so that pixels from that keyframe can be used
// to search for that point.
// Also stores stuff like inlier/outlier counts, and privat information for 
// both Tracker and MapMaker.

#ifndef __MAP_POINT_H
#define __MAP_POINT_H
#include <TooN/TooN.h>
#include <cvd/image_ref.h>
#include <cvd/timer.h>
#include <set>
#include <boost/smart_ptr.hpp>

#include "TrackerData.h"

namespace ptam{
class KeyFrame;

// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information
 
struct MapMakerData
{
  std::set<boost::weak_ptr<KeyFrame> > sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<boost::weak_ptr<KeyFrame> > sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};


struct MapPoint
{
  // Constructor inserts sensible defaults and zeros pointers.
  inline MapPoint()
  {
    bBad = false;
    bfixed = false;
    nMEstimatorOutlierCount = 0;
    nMEstimatorInlierCount = 0;
    dCreationTime = CVD::timer.get_time();

    nSourceCamera = 0;// by default, its source camera is the master camera.
    nFoundCamera = 0;
    sourceKfIDtransfered = false;
    bUpdated = false;
    mblocked = false;
  };
  
  // Where in the world is this point? The main bit of information, really.
  TooN::Vector<3> v3WorldPos;

  // Is it a dud? In that case it'll be moved to the trash soon.
  bool bBad;
  bool bfixed; // used in dual cam case, trust some points very much
               // and in VO use case, where we fix it when its original source kf has been deleted
  bool mblocked; /// blocked for localization when tracking failure occurs
  
  // What pixels should be used to search for this point?
  // we transfer this identity to a new existing keyframe of the local map which can
  // observe it, when its original source kf is removed, in the full slam system
  boost::weak_ptr<KeyFrame> pPatchSourceKF; // The KeyFrame the point was originally made in
  int nSourceLevel;         // Pyramid level in source KeyFrame
  CVD::ImageRef irCenter;   // This is in level-coords in the source pyramid level
  CVD::ImageRef irCenterZero; /// image-coords of the point in the pyramid level Zero
  // in dual camera case, also label the source kf from which camera it was made. maybe useful
  int nSourceCamera;
  int nFoundCamera; /// In the current frame, from which camera it has been measured
  std::set<boost::weak_ptr<KeyFrame> > pSourceKFs; // all kfs measuring the map point
  bool sourceKfIDtransfered; // if transfered to other source kf, avoid re-send to the backend
  bool refreshed;// refreshed after the source kf id transfer?

  // yang, relative representation to its father source keyframe, used after Graph Pose-optimisation update
  TooN::SE3<> v3SourceKFfromeWorld; // its father source keyframe pose
  TooN::Vector<3> v3RelativePos;      // relative pose in its father keyframe
  bool bUpdated; // whether it has been updated by the global map in the current step.

  // What follows next is a bunch of intermediate vectors - they all lead up
  // to being able to calculate v3Pixel{Down,Right}_W, which the PatchFinder
  // needs for patch warping!
  
  TooN::Vector<3> v3Center_NC;             // Unit vector in Source-KF coords pointing at the patch center
  TooN::Vector<3> v3OneDownFromCenter_NC;  // Unit vector in Source-KF coords pointing towards one pixel down of the patch center
  TooN::Vector<3> v3OneRightFromCenter_NC; // Unit vector in Source-KF coords pointing towards one pixel right of the patch center
  TooN::Vector<3> v3Normal_NC;             // Unit vector in Source-KF coords indicating patch normal
  
  TooN::Vector<3> v3PixelDown_W;           // 3-Vector in World coords corresponding to a one-pixel move down the source image
  TooN::Vector<3> v3PixelRight_W;          // 3-Vector in World coords corresponding to a one-pixel move right the source image
  void RefreshPixelVectors();        // Calculates above two vectors
  
  // Info for the Mapmaker (not to be trashed by the tracker:)
  MapMakerData MMData;
  
  // Info for the Tracker (not to be trashed by the MapMaker:)
  // yang, tracker need to be hacked to be able to tolerate dual camera model
  TrackerData TData;
  
  // Info provided by the tracker for the mapmaker:
  int nMEstimatorOutlierCount;
  int nMEstimatorInlierCount;
  
  // Random junk (e.g. for visualisation)
  double dCreationTime; //timer.get_time() time of creation
};
} // namespace

#endif
