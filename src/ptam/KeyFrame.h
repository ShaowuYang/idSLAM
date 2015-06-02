// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <vector>
#include <set>
#include <map>
#include <sensor_msgs/PointCloud.h>
#include <boost/smart_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>

#include "CameraModel.h"
#include "SmallBlurryImage.h"

#define mMaxDepth 4.0 // maximal depth allowed for using the depth measurement

namespace ptam{

class MapPoint;

#define LEVELS 4
// Candidate: a feature in an image which could be made into a map point
struct Candidate
{
  CVD::ImageRef irLevelPos;
  TooN::Vector<2> v2RootPos;
  double dDepth;
  double dSTScore;
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
struct Measurement
{
  int nLevel;   // Which image level?
  bool bSubPix; // Has this measurement been refined to sub-pixel level?
  TooN::Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
  double dDepth;   // Measured depth in [m] or 0 if not available
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

// Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
// This contains image data and corner points.
struct Level
{
  inline Level()
  {
    bImplaneCornersCached = false;
  };
  
  CVD::Image<CVD::byte> im;                // The pyramid level pixels
  std::vector<CVD::ImageRef> vCorners;     // All FAST corners on this level
  std::vector<double> vCornersDepth;
  std::vector<int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
  std::vector<CVD::ImageRef> vMaxCorners;  // The maximal FAST corners
  std::vector<double> vMaxCornersDepth; // valid 3d positions

  Level& operator=(const Level &rhs);
  
  std::vector<Candidate> vCandidates;   // Potential locations of new map points
  
  bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
  std::vector<TooN::Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates
};

struct TimeStamp
{
    TimeStamp() : sec(0), nsec(0) {}
    TimeStamp(int sec, int nsec) : sec(sec), nsec(nsec) {}
    int sec;
    int nsec;
};

enum EdgeType {EDGE_PTAM, EDGE_MOTIONMODEL, EDGE_LOOP, EDGE_INIT};

struct Edge
{
    Edge(int idA, int idB, EdgeType type, const Sophus::SE3d& aTb) :
        idA(idA), idB(idB), type(type), aTb(aTb), valid(true), dSceneDepthMean(1.0) {}
    Edge() : valid(false) {}
    int idA;
    int idB;
    Sophus::SE3d aTb;

    EdgeType type;

    bool valid;

    double dSceneDepthMean;      // Hacky hueristics to improve the information matrix of edges in pose graph

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & idA & idB & aTb & type & valid;
    }
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these 
// are then simply discarded, but sometimes they're then just added to the map.
struct KeyFrame
{
  inline KeyFrame()
  {
      nSourceCamera = 0;
      mAssociateKeyframe = false;
      mAssociatedinFinalQueue = false;
      id = -1;
      updatedByGMap = false;
      SentToGMap = false;
      pointSent = false;
      rgbIsBgr_ = false;
      finalized = false;
      finalizGoodkf = false;
      mbKFlocked = false;
  }
  virtual ~KeyFrame() {};
  int id; // for a mappoint, it is the id of its source keyframe, it has to be updated whenever its current
          // source kf is removed from the local map, to a oldest existing kf in the local map
  bool updatedByGMap; // has been updated according to gmap?
  bool SentToGMap;// has this kf already sent to the gmap?
  bool pointSent;// its associated map points sent to the gmap?

  TooN::SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation
  bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)
  Level aLevels[LEVELS];  // Images, corners, etc lives in this array of pyramid levels
//  Level aLevels_sec[LEVELS];  // Images, corners, etc lives in this array of pyramid levels
  std::map<boost::shared_ptr<MapPoint>, Measurement> mMeasurements;           // All the measurements associated with the keyframe
  
  void MakeKeyFrame_Lite(CVD::BasicImage<CVD::byte> &im, int nCam = 0);   // This takes an image and calculates pyramid levels etc to fill the
                                                            // keyframe data structures with everything that's needed by the tracker..
  void MakeKeyFrame_Rest();                                 // ... while this calculates the rest of the data which the mapmaker needs.
  void MakeKeyFrame(CVD::BasicImage<CVD::byte> &im, CVD::BasicImage<uint16_t> &depth,CameraModel* cam); // Try & extract 3d positions for all non max FAST corners
  // Version for sparse stereo data
  void MakeKeyFrame(CVD::BasicImage<CVD::byte> &im, const sensor_msgs::PointCloud& points, CameraModel* cam);

  double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.
  double dSceneDepthSigma;
  
  //eth{
  double dSceneDepthMedian;	// used to keep same scale after auto-re-init

  SmallBlurryImage SBI; // The relocaliser uses this
  SmallBlurryImage SBIsec; // The relocaliser uses this

  TimeStamp time;

  bool bComplete; // true if this KF is completely initialized
                  // false otherwise, i.e. MakeKeyFrame_Rest still needs to be called
  bool bNewsec; /// a new kf from additional cameras? useful when handling losing images from some cameras of the multi-cam system
  // for landing pad detection
  bool islandingpadDetected;
  std::vector<cv::Point> mPadCorners;// image coordinate of the 4 landing pad corners
  std::vector<boost::shared_ptr<MapPoint> > mMapPointsInPad;// those map points located in the landing pad
  TooN::SE3<> mSe3Landingpadfromworld;
  TooN::Vector<3> mLandingpadNormal;
  std::vector<TooN::Vector<3> > mPadCornersWorld;// mainly used for visualization
  TooN::Vector<3> mPadCenterWorld;// world coordinate of the landing pad center

  // for RGBD
  CVD::Image<uint16_t> depthImage; // Depth image
  CVD::Image<CVD::Rgb<CVD::byte> > rgbImage; // RGB image
//  cv::Mat depthImage; // Depth image
//  cv::Mat rgbImage; // RGB image
  bool rgbIsBgr_;

  // only for dual camera case
  int nSourceCamera; // camera number
  bool mAssociateKeyframe;// associated with a keyframe from another camera?
  bool mAssociatedinFinalQueue;// better named mAssociatedinMap; whether its associated kf already been put in the final map kf queue?
  uint nAssociatedKf;// the serial number of the associated kf in its kf queue of the map
                // it would have been much simpler if we use data structure of "map"!!
  TooN::SE3<> se3Cam2fromCam1; // store each transformation, when assuming dual cameras

  /// only for the back-end processes ///////////////
  void finalizeKeyframeBackend(); /// further process (compute descriptors of) the keyframe for the loop cloure detection
  void finalizeKeyframekpts(); /// further process (compute descriptors of the corners of) the keyframe for relocalization
  bool finalized; /// finalized for the loop closing
  std::vector<boost::shared_ptr<MapPoint> > mapPoints;  /// measured map points in this keyframe
  std::vector<cv::KeyPoint> mpKeypoints;
  cv::Mat mpDescriptors; /// descriptors for map points
  cv::Mat kpDescriptors; /// for all corners
  std::vector<cv::KeyPoint> keypoints; /// all corners
  std::vector<float> kpDepth; /// Depth values at keypoint locations or 0 if unavailable.

  /// for relocalization
  void finalizeKeyframeGoodkf(); /// we only care about the zero-level features for relocalization
  bool finalizGoodkf;
  std::vector<boost::shared_ptr<MapPoint> > mapPointsFirstLevel;
  std::vector<cv::KeyPoint> mpFirstKeypoints;
  cv::Mat mpFDescriptors;
  bool mbKFlocked; /// kf locked after failure

  /// for debug only////
  cv::Mat cvImgDebug;
  //////////////////////

  typedef std::map<int, boost::shared_ptr<Edge> > EdgeMap;
  EdgeMap edges; // Outgoing edges to other keyframes
  std::multimap<double, int> neighbor_ids_ordered_by_distance;

  ////////////////////////////////////////////////////

private:
  // Creates the row lookup table for the given pyramid level
  void createRowLookupTable(int level);
//  void createRowLookupTable_sec(int level);
};

typedef std::map<boost::shared_ptr<MapPoint>, Measurement>::iterator meas_it;  // For convenience, and to work around an emacs paren-matching bug
typedef std::map<boost::shared_ptr<MapPoint>, Measurement>::const_iterator const_meas_it;  // For convenience, and to work around an emacs paren-matching bug

} // namespace

#endif
