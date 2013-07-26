#ifndef MESHMAKER_H
#define MESHMAKER_H

#include "CloudProcessor.h"
#include "types.h"

class MeshMaker
{
protected:


public:
  MeshMaker();
  void poissonReconstruction(PointList & points, Polyhedron & poly, const int nbNeig, FT sm_angle, FT sm_radius, FT sm_distance);

};

#endif
