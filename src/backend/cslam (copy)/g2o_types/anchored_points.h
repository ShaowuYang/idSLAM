// This file is part of ScaViSLAM.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// ScaViSLAM is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// ScaViSLAM is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with ScaViSLAM.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SCAVISLAM_G2O_ANCHORED_POINTS_H
#define SCAVISLAM_G2O_ANCHORED_POINTS_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>

#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

#include "global.h"

namespace cslam
{

using namespace Eigen;
using namespace cslam;
using namespace Sophus;

class G2oVertexSim3 : public g2o::BaseVertex<6, Sim3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  G2oVertexSim3              ();

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  virtual void
  oplus                      (double * update);
  Vector2d
  cam_map                    (const Vector2d & v) const;

  virtual void
  setToOrigin                () {}

  Vector2d principle_point;
  double focal_length;
};

class G2oEdgeSim3
    : public g2o::BaseBinaryEdge<6, Sim3, G2oVertexSim3, G2oVertexSim3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  G2oEdgeSim3                (){}

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  void computeError          ();

};

}

#endif
