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

#include "anchored_points.h"

//#include "transformations.h"

namespace cslam
{
G2oVertexSim3
::G2oVertexSim3()
  : principle_point(Vector2d(0., 0.)),
    focal_length(1.)
{
}

void G2oVertexSim3
::oplus(double * update_p)
{
  Map<Vector7d> update(update_p);
  estimate() = Sim3::exp(update)*estimate();
}

Vector2d  G2oVertexSim3
::cam_map(const Vector2d & v) const
{
  Vector2d res;
  res[0] = v[0]*focal_length + principle_point[0];
  res[1] = v[1]*focal_length + principle_point[1];
  return res;
}

//TODO: implement, but first remove camera parameters from vertex state
bool G2oVertexSim3
::write(std::ostream& os) const
{
  assert(false);
  return true;
}

//TODO: implement, but first remove camera parameters from vertex state
bool G2oVertexSim3
::read(std::istream& is)
{
  assert(false);
  return true;
}

bool G2oEdgeSim3
::read(std::istream& is)
{
  Vector7d v7;
  for (int i=0; i<7; i++)
  {
    is >> v7[i];
  }
  Sim3 cam2world  = Sim3::exp(v7);
  measurement() = cam2world.inverse();
  inverseMeasurement() = cam2world;
  for (int i=0; i<7; i++)
    for (int j=i; j<7; j++)
    {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i) = information()(i,j);
    }
  return is;
}

bool G2oEdgeSim3
::write(std::ostream& os) const
{
  Sim3 cam2world(measurement().inverse());
  Vector7d v7 = cam2world.log();
  for (int i=0; i<7; i++)
  {
    os  << v7[i] << " ";
  }
  for (int i=0; i<7; i++)
    for (int j=i; j<7; j++){
      os << " " <<  information()(i,j);
    }
  return true;
}


void G2oEdgeSim3
::computeError()
{
  const G2oVertexSim3* v1 = static_cast<const G2oVertexSim3*>(_vertices[0]);
  const G2oVertexSim3* v2 = static_cast<const G2oVertexSim3*>(_vertices[1]);
  Sim3 C(_measurement);
  Sim3 error_= v2->estimate().inverse()*C*v1->estimate();
  _error = error_.log().head<6>();
}

}




















