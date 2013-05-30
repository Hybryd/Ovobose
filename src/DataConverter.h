#ifndef DATACONVERTER_H
#define DATACONVERTER_H

#include <cv.h>
#include <fstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <string>

// Read XYZ file 
// Convert data in XYZ file to pcl::PointCloud<pcl::PointXYZ>
// 


class DataConverter
{
protected:
//  pcl::PointCloud<pcl::PointXYZ>::Ptr data; // default format
//  bool empty;

public:
  DataConverter();
//  DataConverter(std::string fileName);
//  DataConverter(pcl::PointCloud<pcl::PointXYZ>::Ptr pData);
//  DataConverter(std::vector<cv::Mat> & vecMat);
  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPointXYZ();
  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
//  bool isEmpty() {return empty;}
  
  void convert(std::vector<cv::Mat> & vecMat, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void convert(std::vector<cv::Mat> & vecMat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
  void read(std::string fileName, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void read(std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
  void save(std::string fileName, std::vector<cv::Mat> & data);
  void save(std::string fileName, pcl::PointCloud<pcl::PointXYZ> & cloud);
  void save(std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};

#endif
