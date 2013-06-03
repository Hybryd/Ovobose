#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

/*!
*
* \file   PostProcessor.h
* \brief  Header file of PostProcessor class.
*
*/

// Conflict between OpenCV and PCL about FLAN. Here we don't implement smooth

//#include <cv.h>
#include <fstream>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/vtk.h>
#include <pcl/surface/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_utils.h>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>


/*!
*
* \class PostProcessor
* \brief Contains a set of functions that remove points lying outside a domain and smooth points
*
*/

class PostProcessor
{
protected:


public:

  PostProcessor();

  void smoothMesh(pcl::PolygonMesh::Ptr meshIn, pcl::PolygonMesh & meshOut, unsigned long int nbIter, double convergence, double relaxFactor, bool edgeSmoothing, bool angle, bool boundarySmoothing);
//  void smooth(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, bool polygonalFit, double radius);
  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ> & dataRaw, pcl::PointCloud<pcl::PointXYZ> & dataPP, pcl::PointXYZ center, double radius, double height);
  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, pcl::PointXYZ center, double radius, double height);

};

#endif
