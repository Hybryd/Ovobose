#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <cv.h>
#include <fstream>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <string>


class PostProcessor
{
protected:
//  std::string                             inputFile;  // contains raw data
//  std::string                             outputFile; // contains processed data
//  pcl::PointCloud<pcl::PointXYZ>::Ptr     data;       // raw data
//  pcl::PointCloud<pcl::PointXYZ>::Ptr     dataPP;     // processed data


public:
  PostProcessor();
//  PostProcessor(std::string inFile, std::string outFile);
//  PostProcessor(pcl::PointCloud<pcl::PointXYZ>::Ptr pData, std::string outFile);
  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr getDataPP() {return dataPP;}
  
//  void read();
  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ> & dataRaw, pcl::PointCloud<pcl::PointXYZ> & dataPP, pcl::PointXYZ center, double radius, double height);
  void keepInCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, pcl::PointXYZ center, double radius, double height);
  
//  void saveAsPCD();
//  void saveAsPLY();
//  void saveAsXYZ();
//  void save();

};

#endif
