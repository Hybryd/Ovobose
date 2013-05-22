#ifndef SCAN_H
#define SCAN_H

#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846264

////////////////////////////////////////////////////////////////////
// Read parameters from a file and scan the laser line to compute //
// 3D corresponding coordinates.                                  //
// The output file is aimed to be read by gnuplot                 //
//                                                                //
////////////////////////////////////////////////////////////////////

class Scan
{
protected:
  std::string                               paramFile;
  std::string                               outputFile;
  int                                       Vcut;       // value for the manual thresholding
  std::vector<cv::Mat>                      data;       // the set of all measures (vector of vector of points)
  cv::Mat                                   matM;       // transformation matrix
  cv::Mat                                   vecN;       // normal vector to the laser plane
  double                                   angle;      // current angle for rotation
  
public:
  Scan();
  Scan(std::string pFile, std::string outFile, int pVcut);
  std::vector<cv::Mat> & getData() {return data;}
  
  void read(std::string pNameMatrix, std::string pNameNormal);
  
  void launch();  // called to take several measures (during a rotation for instance)
  cv::Mat makeRotationMatrix();
  void measure(cv::Mat & current); // called when ready to measure
  
  void save();

};


#endif

