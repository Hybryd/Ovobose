#ifndef CLOUDPROCESSOR_H
#define CLOUDPROCESSOR_H

/*!
*
* \file   CloudProcessor.h
* \brief  Header file of CloudProcessor class.
*
*/


#include <cv_compatible.h>
#include <fstream>
#include <iostream>
#include <string>
//#include <vtkSmartPointer.h>
//#include <vtkSmoothPolyDataFilter.h>

#include "types.h"

/*!
*
* \class CloudProcessor
* \brief Contains a set of functions that remove points lying outside a domain, smoothing cloud and meshes algorithms
*
*/

class CloudProcessor
{
protected:


public:

  CloudProcessor();

  void removeOutliers(PointList & points, const double removed_percentage, const int nb_neighbors);
  void simplifyCloud(PointList & points, double cell_size);
  void smoothCloud(PointList & points, int iter, const int nb_neighbors);
  void estimateNormals(PointList & points, PointWNList & pointsWN, const int nb_neighbors);
  void orientNormals(PointWNList & pointsWN, const int nb_neighbors);
  FT computeAverageSpacing(PointList & points);

//  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ> & dataRaw, pcl::PointCloud<pcl::PointXYZ> & dataPP, pcl::PointXYZ center, double radius, double height);
//  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, pcl::PointXYZ center, double radius, double height);

};

#endif
