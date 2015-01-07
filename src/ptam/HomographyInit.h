// -*- c++ -*- 
// Copyright 2008 Isis Innovation Limited

// HomographyInit.h 
// Declares the HomographyInit class and a few helper functions. 
//
// This class is used by MapMaker to bootstrap the map, and implements
// the homography decomposition of Faugeras and Lustman's 1988 tech
// report.
//
// Implementation according to Faugeras and Lustman

#ifndef __HOMOGRAPHY_INIT_H
#define __HOMOGRAPHY_INIT_H
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <vector>

// Homography matches are 2D-2D matches in a stereo pair, unprojected
// to the Z=1 plane.
struct HomographyMatch
{
  // To be filled in by MapMaker:
  TooN::Vector<2> v2CamPlaneFirst;
  TooN::Vector<2> v2CamPlaneSecond;
  TooN::Matrix<2> m2PixelProjectionJac;
};

// Storage for each homography decomposition
struct HomographyDecomposition
{
  TooN::Vector<3> v3Tp;
  TooN::Matrix<3> m3Rp;
  double d;
  TooN::Vector<3> v3n;
  
  // The resolved composition..
  TooN::SE3<> se3SecondFromFirst;
  int nScore;
};

class HomographyInit
{
public:
  bool Compute(std::vector<HomographyMatch> vMatches, double dMaxPixelError, TooN::SE3<> &se3SecondCameraPose);

  //use circle pattern for initialization
  bool TransIniFromCircle(TooN::SE3<> se3First, TooN::SE3<> se3Second, double disdiffmin, double &transSecondFromFirst);

protected:
  TooN::Matrix<3> HomographyFromMatches(std::vector<HomographyMatch> vMatches);
  void BestHomographyFromMatches_MLESAC();
  void DecomposeHomography();
  void ChooseBestDecomposition();
  void RefineHomographyWithInliers();
  
  bool IsHomographyInlier(TooN::Matrix<3> m3Homography, HomographyMatch match);
  double MLESACScore(TooN::Matrix<3> m3Homography, HomographyMatch match);
  
  double mdMaxPixelErrorSquared;
  TooN::Matrix<3> mm3BestHomography;
  std::vector<HomographyMatch> mvMatches;
  std::vector<HomographyMatch> mvHomographyInliers;
  std::vector<HomographyDecomposition> mvDecompositions;
};


#endif
