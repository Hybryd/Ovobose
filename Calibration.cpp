#include "Calibration.h"


Calibration::Calibration()
{
  cptMouse = 0;
  paramFile = "param.xml";
  cvNamedWindow( "Calibration" );
}

Calibration::Calibration(std::string pFile)
{
  cptMouse = 0;
  if (pFile.size() > 4 && pFile.find(".xml")==pFile.size()-4)
  {
//    /// ADEL
//    std::cerr << "contructor" << std::endl;
//    /// 
    paramFile = pFile;
  }
  else
  {
    std::cerr << "Warning: bad name for output file. Parameters will be saved in \"param.xml\"." << std::endl;
    paramFile = "param.xml";
  }
  cvNamedWindow( "Calibration" );
}

Calibration::~Calibration()
{
}



void onMouseCircularPattern(int event, int x, int y, int flags, void* pair)
{
  std::pair<Calibration *, double> * lPair = (std::pair<Calibration *, double> *) pair;
  Calibration * cal = lPair->first;
  double radius = lPair->second;
  int & cptMouse = cal-> getCptMouse();
//  double radius = cal->getRadius;
  if(event == CV_EVENT_LBUTTONDOWN)
  {
    cal->getVecPoints().push_back(cv::Point(x,y));
    cal->getVecRPoints().push_back(cv::Point3d(radius*sin(cptMouse*10*PI/180),0,radius*(1-cos(cptMouse*10*PI/180))));
    cptMouse++;
  }
}

//
// pNbCal  : number of calibration points
// pRadius : radius of the circular pattern
//
void Calibration::circularPattern(int pNbCal, double pRadius)
{
  std::pair<Calibration*,double> pair;
  pair.first=this;
  pair.second=pRadius;
  cv::setMouseCallback("Calibration", onMouseCircularPattern, &pair);
  launch(pNbCal);
//  int nbCal=19;        // number of calibration points
//  double radius=5.6;  // radius of the circular pattern in cm

}


void Calibration::launch(int pNbCal)
{
  std::stringstream s;
  
  cv::VideoCapture capture = cv::VideoCapture(0);
  if(!capture.isOpened())
  {
    std::cerr << "Error: unable to open capture video." << std::endl;
    exit(-1);
  }
  capture >> image;
  
  while(cptMouse < pNbCal)
  {
    s.str("");
    s << "Point " << cptMouse << "/" << pNbCal << " (" << (cptMouse*10) << " deg)";
    cv::putText(image, s.str(), cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,200), 1);  
    
    cv::waitKey(10);
    imshow("Calibration", image);
    capture >> image;
  }
  
  // Construct the linear system
  cv::Mat matMM(2*pNbCal,11,cv::DataType<double>::type);
	cv::Mat vecU(2*pNbCal,1,cv::DataType<double>::type);
  double u=0;
  double v=0;
  cv::Point3d realP(0,0,0); // corresponding 3D point in reality

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
  matM.create(3,4,cv::DataType<double>::type);
  for(int i=0;i<11;++i)
  {
    matM.at<double>(i/4,i%4) = vecM.at<double>(i,0);
  }
  matM.at<double>(2,3) = 1; // m34 = 1

  // Construct the normal vector to the laser plane
  vecN.create(4,1,cv::DataType<double>::type); // Hard coded normal vector : [0,-1,0,0]
  vecN.at<double>(0,0) = 0;
  vecN.at<double>(1,0) = -1;
  vecN.at<double>(2,0) = 0;
  vecN.at<double>(3,0) = 0;

}


void Calibration::save(std::string pNameMatrix, std::string pNameNormal)
{
  cv::FileStorage fs(paramFile, cv::FileStorage::WRITE);
//  std::cout << "    Saving transformation matrix." << std::endl;
  std::string lM = pNameMatrix;
  std::string lN = pNameNormal;
  
  if(pNameMatrix.size()==0 || pNameNormal.size()==0 || pNameMatrix.find(" ") != std::string::npos || pNameNormal.find(" ") != std::string::npos)
  {
    std::cerr << "Warning: bad name for data. The transformation matrix will be saved as \"matM\" and the normal vector as \"vecN\"." << std::endl;
    lM = "matM";
    lN = "vecN";
  }
  
  fs << lM << matM;
  fs << lN << vecN;
  
  fs.release();
  
}




