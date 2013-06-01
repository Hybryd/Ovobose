#ifndef CALIBRATION_H
#define CALIBRATION_H

/*!
*
* \file   Calibration.h
* \brief  Header file of Calibration class.
*
*/


#include <cmath>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846264


  ////////////////////////////////////////////////////////////////
  // Find the transformation matrix from the calibration points //
  ////////////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////////////////////
  // Transformation matrix matM:                               //
  ///////////////////////////////////////////////////////////////
  //                                                           //
  //                         |X|                               //
  // |s.u| |m11 m12 m13 m14| |Y|                               //
  // |s.v|=|m21 m22 m23 m24|.|Z|                               //
  // | s | |m31 m32 m33  1 | |1|                               //
  //                                                           //
  // System to solve :                                         //
  //                                                           //
  // |X1 Y1 Z1 1  0  0  0  0 -u1.X1 -u1.Y1 -u1.Z1| |m11| |u1|  //
  // |0  0  0  0  X1 Y1 Z1 1 -v1.X1 -v1.Y1 -v1.Z1| |m12| |v1|  //
  // |X2 Y2 Z2 1  0  0  0  0 -u2.X2 -u2.Y2 -u2.Z2| |m13| |u2|  //
  // |0  0  0  0  X2 Y2 Z2 1 -v2.X2 -v2.Y2 -v2.Z2| |m14| |v2|  //
  //                       :                         :    :    //
  // |XN YN ZN 1  0  0  0  0 -uN.XN -uN.YN -uN.ZN|.|m32|=|uN|  //
  // |0  0  0  0  XN YN ZN 1 -vN.XN -vN.YN -vN.ZN| |m33| |vN|  //
  //                                                           //
  //                        matMM                 . vecM= vecU //
  //                                                           //
  ///////////////////////////////////////////////////////////////


/*!
*
* \class Calibration
* \brief Manages calibration process to compute the transformation matrix from 3D point to their corresponding points on the picture. This matrix and the leaser plane equation (hardcoded) are stored in a XML file.
*
*/


class Calibration
{
protected:
  int                       cptMouse;     // mouse clicks counter
  std::string               paramFile;    // name of the output file containing parameters
  cv::Mat                   image;        // image displayed
  std::vector<cv::Point>    vecPoints;    // calibration points in image
  std::vector<cv::Point3d>  vecRPoints;   // calibration points in reality
  cv::Mat                   matM;         // transformation matrix
  cv::Mat                   vecN;         // normal vector to the laser plane

public:
  Calibration();
  Calibration(std::string pFile);
  ~Calibration();
  
  int & getCptMouse() {return cptMouse;}
  std::vector<cv::Point> &  getVecPoints() {return vecPoints;}
  std::vector<cv::Point3d> &  getVecRPoints() {return vecRPoints;}
  
//  void onMouseCircularPattern(int event, int x, int y, int flags, void* param);
  void circularPattern(int pNbCal,double pRadius);
  
  void launch(int pNbCal);
  
  void save(std::string pNameMatrix, std::string pNameNormal);
  
};


#endif
