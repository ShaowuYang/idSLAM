// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __TRACKERDATA_H
#define __TRACKERDATA_H

#include "PatchFinder.h"
#include "CameraModel.h"
// This class contains all the intermediate results associated with
// a map-point that the tracker keeps up-to-date. TrackerData
// basically handles all the tracker's point-projection jobs,
// and also contains the PatchFinder which does the image search.
// It's very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

namespace ptam{
class MapPoint;

struct TrackerData
{
  PatchFinder Finder;
  
  // Projection itermediates:
  Vector<3> v3Cam;        // Coords in current cam frame
  Vector<2> v2ImPlane;    // Coords in current cam z=1 plane
  Vector<2> v2Image;      // Pixel coords in LEVEL0
  Matrix<2> m2CamDerivs;  // Camera projection derivs
  bool CamDerivesGot;     // yang, only need to get once
  bool bInImage;
  bool bInImageSec; // in the second img
  bool bPotentiallyVisible;
  
  int nSearchLevel;
  bool bSearched;
  bool bFound;
  bool bDidSubPix;
  Vector<2> v2Found;      // Pixel coords of found patch (L0)
  double dFoundDepth;
  Vector<3> v3Found;
  double dSqrtInvNoise;   // Only depends on search level..
  double dSqrtInvDepthNoise;

  // Stuff for pose update:
  Vector<2> v2Error_CovScaled;
  Matrix<2,6> m26Jacobian;   // Jacobian wrt camera position
  Matrix<2,12> m212JacobianWE;   // Jacobian wrt camera position, with error included

  Vector<3> v3Error_CovScaled;
  Matrix<3,6> m36Jacobian;   // Jacobian wrt camera position
  
  // Project point into image given certain pose and camera.
  // This can bail out at several stages if the point
  // will not be properly in the image.
  inline void Project(const TooN::Vector<3>& point, const SE3<> &se3CFromW, CameraModel* Cam)
  {
    bInImage = bPotentiallyVisible = false;
    v3Cam = se3CFromW * point;
    if(v3Cam[2] < 0.001)
      return;
    v2ImPlane = project(v3Cam);
    if(v2ImPlane*v2ImPlane > Cam->LargestRadiusInImage() * Cam->LargestRadiusInImage())
      return;
    v2Image = Cam->Project(v2ImPlane);
    if(Cam->Invalid())
      return;
    
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
      return;
    bInImage = true;
  }
  
  // Get the projection derivatives (depend only on the camera.)
  // This is called Unsafe because it depends on the camera caching 
  // results from the previous projection:
  // Only do this right after the same point has been projected!
  inline void GetDerivsUnsafe(CameraModel* Cam) 
  {
    m2CamDerivs = Cam->GetProjectionDerivs();
    CamDerivesGot = true;
  }
  
  // Does projection and gets camera derivs all in one.
  inline void ProjectAndDerivs(const TooN::Vector<3>& point, SE3<> &se3, CameraModel* Cam)
  {
    Project(point, se3, Cam);
    if(bFound)
      GetDerivsUnsafe(Cam);
  }
  
  // Jacobian of projection W.R.T. the camera position
  // I.e. if  p_cam = SE3Old * p_world, 
  //         SE3New = SE3Motion * SE3Old
  // J = Jh * (I3*3 | -[pc]*)
  inline void CalcJacobian()
  {
    double dOneOverCameraZ = 1.0 / v3Cam[2];
    for(int m=0; m<6; m++)
      {
	const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
	Vector<2> v2CamFrameMotion;
	v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	m26Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion;
      };

    if (dFoundDepth > 0.0) {
        for(int m=0; m<6; m++)
          {
            const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
            Vector<3> v3CamFrameMotion;
            v3CamFrameMotion[0] = v4Motion[0] * dOneOverCameraZ;
            v3CamFrameMotion[1] = v4Motion[1] * dOneOverCameraZ;
            v3CamFrameMotion[2] = v4Motion[2] * dOneOverCameraZ;
            m36Jacobian.T()[m] = v3CamFrameMotion;
          };
    }
  }
  
  // in the dual camera case, Jacobian w.r.t the second camera have to be calculated differently,
  // due to the additional transformation of cam1 to cam2
  // see equ. 10.10 in tech. rep. from JL. Blanco
  // J1 = R(A) (I3*3 | -[pc1]*)
  // J2 = Jh * J1
  inline void CalcJacobiansec(const SE3<> mse3cam2fromcam1)
  {
    double dOneOverCameraZ = 1.0 / v3Cam[2];
    Vector<3> v3Cam1 = mse3cam2fromcam1.inverse()*v3Cam;// important note, here we need the pose of point in cam1!
    for(int m=0; m<6; m++)
      {
    const Vector<4> v4Motion = generate_J(m, unproject(v3Cam1), mse3cam2fromcam1.get_rotation().get_matrix());
    Vector<2> v2CamFrameMotion;
    v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
    v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
    m26Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion;
      };

    if (dFoundDepth > 0.0) {
        for(int m=0; m<6; m++)
          {
            const Vector<4> v4Motion = generate_J(m, unproject(v3Cam1), mse3cam2fromcam1.get_rotation().get_matrix());
            Vector<3> v3CamFrameMotion;
            v3CamFrameMotion[0] = v4Motion[0] * dOneOverCameraZ;
            v3CamFrameMotion[1] = v4Motion[1] * dOneOverCameraZ;
            v3CamFrameMotion[2] = v4Motion[2] * dOneOverCameraZ;
            m36Jacobian.T()[m] = v3CamFrameMotion;
          };
    }
  }

  inline void CalcJacobianWithError()
  {
      double dOneOverCameraZ = 1.0 / v3Cam[2];
      for(int m=0; m<6; m++)
      {
          const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
          Vector<2> v2CamFrameMotion;
          v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
          v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
          m212JacobianWE.T()[m] = m2CamDerivs * v2CamFrameMotion;
      };
      for(int m=0; m<6; m++)
      {
          m212JacobianWE.T()[m+6] = Zeros;
      };
  }

  inline void CalcJacobianWithErrorSec(const SE3<> mse3cam2fromcam1)
  {
      double dOneOverCameraZ = 1.0 / v3Cam[2];
      Vector<3> v3Cam1 = mse3cam2fromcam1.inverse()*v3Cam;// important note, here we need the pose of point in cam1!
//      Vector<3> v3Camsec = (mse3cam2fromcam1) * v3Cam1;// in the second cam frames
      Vector<3> v3Camsec = v3Cam;
      for(int m=0; m<6; m++)
      {
          const Vector<4> v4Motion = generate_J(m, unproject(v3Cam1), mse3cam2fromcam1.get_rotation().get_matrix());
          Vector<2> v2CamFrameMotion;
          v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
          v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
          m212JacobianWE.T()[m] = m2CamDerivs * v2CamFrameMotion;
      };
      for(int m=0; m<6; m++)
      {
          const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Camsec));
          Vector<2> v2CamFrameMotion;
          v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
          v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
          m212JacobianWE.T()[m+6] = m2CamDerivs * v2CamFrameMotion;
      };
  }

  // get the Jacobian matrix, not considering h(Pc) yet
  // similar to SE3<>::generator_field, but front multiplied with R(A) in it.
  inline Vector<4> generate_J(int i, Vector<4> pos, Matrix<3> R){
      Vector<4> result(Zeros);
      if(i < 3){
          result[0]=R(0, i);
          result[1]=R(1, i);
          result[2]=R(2, i);
        return result;
      }
      switch(i){
      case 3:
          result[0]=-R(0, 1)*pos[2]+R(0, 2)*pos[1];
          result[1]=-R(1, 1)*pos[2]+R(1, 2)*pos[1];
          result[2]=-R(2, 1)*pos[2]+R(2, 2)*pos[1];
          break;
      case 4:
          result[0]=R(0, 0)*pos[2]-R(0, 2)*pos[0];
          result[1]=R(1, 0)*pos[2]-R(1, 2)*pos[0];
          result[2]=R(2, 0)*pos[2]-R(2, 2)*pos[0];
          break;
      case 5:
          result[0]=-R(0, 0)*pos[1]+R(0, 1)*pos[0];
          result[1]=-R(1, 0)*pos[1]+R(1, 1)*pos[0];
          result[2]=-R(2, 0)*pos[1]+R(2, 1)*pos[0];
          break;
      default:
          break;
      }
      return result;
  }

  // Sometimes in tracker instead of reprojecting, just update the error linearly!
  inline void LinearUpdate(const Vector<6> &v6)
  {
    v2Image += m26Jacobian * v6;
  }
  
  // This static member is filled in by the tracker and allows in-image checks in this class above.
  static CVD::ImageRef irImageSize;
};

}
#endif
