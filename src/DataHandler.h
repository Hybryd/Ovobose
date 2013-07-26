#ifndef DATAHANDLER_H
#define DATAHANDLER_H

/*!
*
* \file   DataHandler.h
* \brief  Header file of DataHandler class.
*
*/


#include <cv_compatible.h>
#include <fstream>
#include <string>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include "types.h"

/*!
*
* \class DataHandler
* \brief Contains a set of functions converting data types, from std::vector<cv::Mat> to pcl::PointCloud<pcl::PointXYZ>, std::vector<cv::Mat> to pcl::PointCloud<pcl::PointXYZ>::Ptr. It also allows creating pcl::PointCloud<pcl::PointXYZ> or pcl::PointCloud<pcl::PointXYZ>::Ptr cloud from a PCD, PLY or XYZ file and saving a cloud in such file types.
*
*/

class DataHandler
{
protected:

public:
  DataHandler();
  
  void readXYZ(std::string fileName, PointList & cloud);
  void saveAsXYZ(std::string fileName, std::vector< std::vector<double> > & vecMat);
  void saveAsXYZ(std::string fileName, PointList & cloud);
  void saveAsXYZ(std::string fileName, PointWNList & cloudWN);
  void saveAsOFF(std::string fileName, CGAL::Polyhedron_3<Kernel> & poly);
  void saveAsOFF(std::string fileName, CGAL::Surface_mesh_complex_2_in_triangulation_3<CGAL::Surface_mesh_default_triangulation_3> & cplx);

//  void saveInXML(std::string fileName, std::vector<std::string> varNames, std::vector<cv::Mat> vars );

};

#endif
