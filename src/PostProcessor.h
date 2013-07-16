#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

/*!
*
* \file   PostProcessor.h
* \brief  Header file of PostProcessor class.
*
*/


#include <cv_compatible.h>
#include <fstream>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
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
* \brief Contains a set of functions that remove points lying outside a domain, smoothing cloud and meshes algorithms
*
*/

class PostProcessor
{
protected:


public:

  PostProcessor();

  void smoothMesh(pcl::PolygonMesh::Ptr meshIn, pcl::PolygonMesh & meshOut, unsigned long int nbIter, double convergence, double relaxFactor, bool edgeSmoothing, double angle, bool boundarySmoothing);
  void smoothCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, bool polygonalFit, double radius, int nbIter);
  void smoothCloudSOR(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, int meanK, double stdDev);
  
  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ> & dataRaw, pcl::PointCloud<pcl::PointXYZ> & dataPP, pcl::PointXYZ center, double radius, double height);
  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, pcl::PointXYZ center, double radius, double height);

};

#endif
