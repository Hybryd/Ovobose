#ifndef SCAN_H
#define SCAN_H

/*!
*
* \file   Scan.h
* \brief  Header file of scan class.
*
*/

#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "DataConverter.h"

#define PI 3.14159265358979323846264


/*!
*
* \class Scan
* \brief Read parameters from a file and scan the laser line to compute 3D corresponding coordinates.
*
*/

class Scan
{
protected:
  std::string                               paramFile;
  std::string                               outputFile;
  int                                       Vcut;       // value for the manual thresholding
  std::vector<cv::Mat>                      data;       // the set of all measures (vector of cv::Mat)
  cv::Mat                                   matM;       // transformation matrix
  cv::Mat                                   vecN;       // normal vector to the laser plane
  double                                   angle;      // current angle for rotation
  double                                   stepAngle;  // rotation between two scans
  
public:
  Scan();
  Scan(std::string pFile, std::string outFile, double stepAng);
  std::vector<cv::Mat> & getData() {return data;}
  
  void read(std::string pNameMatrix, std::string pNameNormal);
  
  void launch();  // called to take several measures (during a rotation for instance)
  cv::Mat makeRotationMatrix();
  void measure(cv::Mat & current); // called when ready to measure
  
  void save();

};


#endif


