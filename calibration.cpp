#include <cmath>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846264

////////////////////////////////////////////////////////////////////////////
// Calibration of the system.                                             //
//                                                                        //
// Compute the transformation 3x4 matrix from real coordinates to image   //
// coordinates from a semi circular pattern.                              //
// 19 calibration points are plotted on the pattern distant from one to   //
// another of 10°.                                                        //
// The user clicks on the image to make the correspondence, starting from //
// the point at the bottom of the pattern to the top.                     //
//                                                                        //
// Usage : ./calibration [parameterFile.xml]                              //
////////////////////////////////////////////////////////////////////////////



int cptMouse=0;      // number of clicks
int nbCal=19;        // number of calibration points
double radius=5.6;  // radius of the circular pattern in cm

cv::Mat image;
std::vector<cv::Point> vecPoints;     // calibration points in image
std::vector<cv::Point3d> vecRPoints;  // calibration points in reality
std::stringstream s;                  // message to display on the image
    

void onMouseCircularPattern(int event, int x, int y, int flags, void* param)
{
  if(event == CV_EVENT_LBUTTONDOWN)
  {
    vecPoints.push_back(cv::Point(x,y));
    vecRPoints.push_back(cv::Point3d(radius*sin(cptMouse*10*PI/180),0,radius*(1-cos(cptMouse*10*PI/180))));
    cptMouse++;
  }
}


//////////
// Main //
//////////

int main(int argc, char ** argv)
{
  int key=0;

  std::stringstream ss;
  if(argc > 1)
    ss << argv[1];
  else
    ss << "param.xml";
  cv::FileStorage fs(ss.str(), cv::FileStorage::WRITE); // parameters file
  
  
  cv::VideoCapture capture = cv::VideoCapture(0);
  if(!capture.isOpened())
  {
    std::cerr << "Unable to open capture video." << std::endl;
    return -1;
  }
  capture >> image;
  
  
  /////////////////////////////////////////////////////////
  // Detect lines on the image by clicking on their ends //
  // from top to bottom                                  //
  /////////////////////////////////////////////////////////
  
  std::cout << std::endl;
  std::cout << " 1) Click first on the top point, then on the middle one and finally on the bottom point." << std::endl;
  
  
  cvNamedWindow( "Calibration using circular pattern" );
  cv::setMouseCallback("Calibration using circular pattern", onMouseCircularPattern, 0);
  
  
  
  while(cptMouse < nbCal)
  {
    s.str("");
    s << "Point " << cptMouse << "/" << nbCal << " (" << (cptMouse*10) << " deg)";
    cv::putText(image, s.str(), cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,200), 1);  
    
    key = cv::waitKey(10);
    imshow("Calibration using circular pattern", image);
    capture >> image;
  }




  
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

  std::cout << std::endl;
  std::cout << " 2) Computing transformation matrix and the normal vector of the laser plane." << std::endl;
    
	
	cv::Mat matMM(2*nbCal,11,cv::DataType<double>::type);
	cv::Mat vecU(2*nbCal,1,cv::DataType<double>::type);
  double u=0;
  double v=0;
  cv::Point3d realP(0,0,0); // corresponding 3D point in reality



	// Construct the linear system
	for(double i=0;i<vecPoints.size();++i)
	{
	  u=(double)vecPoints[i].x;
	  v=(double)vecPoints[i].y;
	  realP = vecRPoints[i];
	  
	  matMM.at<double>(2*i,0)   = realP.x    ; matMM.at<double>(2*i,1)   = realP.y    ; matMM.at<double>(2*i,2)   = realP.z    ; matMM.at<double>(2*i,3)   = 1;
    matMM.at<double>(2*i,4)   = 0          ; matMM.at<double>(2*i,5)   = 0          ; matMM.at<double>(2*i,6)   = 0          ; matMM.at<double>(2*i,7)   = 0;
    matMM.at<double>(2*i,8)   = -u*realP.x ; matMM.at<double>(2*i,9)   = -u*realP.y ; matMM.at<double>(2*i,10)  = -u*realP.z ;
    matMM.at<double>(2*i+1,0) = 0          ; matMM.at<double>(2*i+1,1) = 0          ; matMM.at<double>(2*i+1,2) = 0          ; matMM.at<double>(2*i+1,3) = 0;
    matMM.at<double>(2*i+1,4) = realP.x    ; matMM.at<double>(2*i+1,5) = realP.y    ; matMM.at<double>(2*i+1,6) = realP.z    ; matMM.at<double>(2*i+1,7) = 1;
    matMM.at<double>(2*i+1,8) = -v*realP.x ; matMM.at<double>(2*i+1,9) = -v*realP.y ; matMM.at<double>(2*i+1,10)= -v*realP.z ;
	
	  vecU.at<double>(2*i,0)   = u;
    vecU.at<double>(2*i+1,0) = v;
	}

  // Solve the system
  cv::Mat vecM;
  cv::solve(matMM,vecU,vecM,cv::DECOMP_SVD && cv::DECOMP_NORMAL);
  
  // Construct the transformation matrix, adding the constraint m34=1
  cv::Mat matM(3,4,cv::DataType<double>::type);
  for(int i=0;i<11;++i)
  {
    matM.at<double>(i/4,i%4) = vecM.at<double>(i,0);
  }
  matM.at<double>(2,3) = 1; // m34 = 1

  // Save data
  std::cout << "    Saving transformation matrix." << std::endl;
  fs << "matM" << matM;
  
  cv::Mat vecN(4,1,cv::DataType<double>::type); // Hard coded normal vector : [0,-1,0,0]
  vecN.at<double>(0,0) = 0;
  vecN.at<double>(1,0) = -1;
  vecN.at<double>(2,0) = 0;
  vecN.at<double>(3,0) = 0;
  
  std::cout << "    Saving normal vector." << std::endl;
  fs << "normal" << vecN;
    
  fs.release();
  
  std::cout << std::endl;
  std::cout << " Done! Parameters are stored in the file \"" << ss.str() << "\"." << std::endl << std::endl;

  return 0;
}



