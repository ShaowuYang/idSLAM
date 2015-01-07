// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not 
// invalidated!

#ifndef __MAP_H
#define __MAP_H
#include <vector>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

namespace ptam{
struct MapPoint;
struct KeyFrame;

typedef boost::function<void(std::vector<boost::shared_ptr<KeyFrame> >)> ErasedAllCbFunction;
typedef boost::function<void(boost::shared_ptr<KeyFrame>)> ErasedKfCbFunction;
typedef boost::function<void(const std::vector<boost::shared_ptr<KeyFrame> >&)> BaDoneCbFunction;

struct Map
{
  Map();
  inline bool IsGood() {return bGood;}
  void Reset();
  
  void EraseBadPoints(bool performLock = true);
  void EraseOldKeyFrames(bool performLock = true);
  void EraseAll();

  ErasedAllCbFunction erasedAllCallback;
  ErasedKfCbFunction erasedKfCallback;
  BaDoneCbFunction baDoneCallback;

  bool SaveMap(const std::string sPath);
  bool LoadMap(const std::string sPath);
  
  std::vector<boost::shared_ptr<MapPoint> > vpPoints;
  std::vector<boost::shared_ptr<KeyFrame> > vpKeyFrames;
  std::vector<boost::shared_ptr<KeyFrame> > vpKeyFramessec;

  bool bGood;

  mutable boost::shared_mutex mutex;

  static void emptyErasedAllCb(std::vector<boost::shared_ptr<KeyFrame> >) {}
  static void emptyErasedKfCb(boost::shared_ptr<KeyFrame>) {}
  static void emptyBaDoneCb(const std::vector<boost::shared_ptr<KeyFrame> >& map) {}

  bool didFullBA;
};

} // namespace

#endif
