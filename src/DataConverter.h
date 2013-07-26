#ifndef DATACONVERTER_H
#define DATACONVERTER_H

/*!
*
* \file   DataConverter.h
* \brief  Header file of DataConverter class.
*
*/


#include "/usr/include/opencv/cv_compatible.h"
#include <fstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/io/ply_lib_io.h>
//#include <pcl/io/stl_lib_io.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <string>


/*!
*
* \class DataConverter
* \brief Contains a set of functions converting data types, from std::vector<cv::Mat> to pcl::PointCloud<pcl::PointXYZ>, std::vector<cv::Mat> to pcl::PointCloud<pcl::PointXYZ>::Ptr. It also allows creating pcl::PointCloud<pcl::PointXYZ> or pcl::PointCloud<pcl::PointXYZ>::Ptr cloud from a PCD, PLY or XYZ file and saving a cloud in such file types.
*
*/

class DataConverter
{
protected:

public:
  DataConverter();
  
  void convert(std::vector<cv::Mat> & vecMat, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void convert(std::vector<cv::Mat> & vecMat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void convert(std::vector< std::vector<double> > & vecMat, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void convert(std::vector< std::vector<double> > & vecMat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
  void read(std::string fileName, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void read(std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
  void save(std::string fileName, std::vector< std::vector<double> > & vecMat);
  void save(std::string fileName, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void save(std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
  void saveAsOBJ(std::string &file_name, pcl::PolygonMesh &poly_mesh, unsigned precision);
  void saveAsSTL(std::string &file_name, pcl::PolygonMesh &poly_mesh, unsigned precision);
  
  void savePolygon(std::string fileName, pcl::PolygonMesh &mesh);
  
  void saveInXML(std::string fileName, std::vector<std::string> varNames, std::vector<cv::Mat> vars );

};

#endif
