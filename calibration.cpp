#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

//////////////////////////////////////////////////////////////////////////
// Calibration of the system. Usage : ./calibration [parameterFile.xml] //
//////////////////////////////////////////////////////////////////////////


// Values for the manual thresholding of images during the laser detection
int V_cut=200;
int H_cut=90;


// Function called by the trackbar
void onTrackbar(int, void* = NULL)
{
}




//////////
// Main //
//////////

int main(int argc, char ** argv)
{
  int key=0;
  cv::Mat image;
  cv::Mat imageWL;
  cv::Mat gray_image;
  cv::Mat gray_image2;
  cv::Mat HSV_image;
  cv::Mat HSV_gray_image;
  cv::Mat HSV_gray_image2;
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
  
  cvNamedWindow( "Calibration" );
  
  

  /////////////////////////////////////
  // Calibration with the chessboard //
  /////////////////////////////////////
  
  std::cout << " 1) Calibrating the intrinsic and extrinsic parameters. " << std::endl;
  
  int numBoards = 1;
  int numCornersHor = 6;
  int numCornersVer = 9;
  int numSquares = numCornersHor * numCornersVer;
  cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
  int successes=1;

  std::vector< std::vector< cv::Point3f > > object_points;
  std::vector< std::vector< cv::Point2f > > image_points;
  std::vector< cv::Point2f > corners;
  
  
if(1==1)
{
  std::vector< cv::Point3f > obj;
  for(int j=0; j<numSquares; ++j)
    obj.push_back(cv::Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

  
  while(key != 'q' && successes <= numBoards)
  {
    cvtColor(image, gray_image, CV_BGR2GRAY);
    bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
      cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray_image, board_sz, corners, found);
      
      image_points.push_back(corners);
      object_points.push_back(obj);
//      std::cerr << "Snaps " << successes << "/" << numBoards << "\r";
      
      successes++;
    }
    cv::putText(gray_image, "Snip snap snapi", cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.45, cvScalar(200,200,200), 1);
    imshow("Calibration", gray_image);

    key = cv::waitKey(500); // wait 0.5 seconds
    if(key == 'q')
      return 0;
    
    capture >> image;
  }
//  std::cout << std::endl;

  cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
  cv::Mat distCoeffs;
  std::vector< cv::Mat > rvecs;
  std::vector< cv::Mat > tvecs;
  
  intrinsic.ptr<float>(0)[0] = 1;
  intrinsic.ptr<float>(1)[1] = 1;
  calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

  // Compute the rotation matrix R in [R|t]
  cv::Mat rotation;
  cv::Rodrigues(rvecs[0],rotation);
  



//  std::cout << "Storing parameters..." << std::endl;
  std::cout << "    Saving parameters. " << std::endl;
  fs << "intrinsic" << intrinsic;
  fs << "distorsion" << distCoeffs;
  fs << "rotation" << rotation;
  fs << "translation" << tvecs[0];
//  std::stringstream ss;
//  for(size_t i=0;i<rvecs.size();++i)
//  {
//    ss << "R" << i;
//    fs << ss.str() << rvecs[i];
//    ss.str("");
//  }
//  
//  for(size_t i=0;i<tvecs.size();++i)
//  {
//    ss << "T" << i;
//    fs << ss.str() << tvecs[i];
//    ss.str("");
//  }
//  
  
  cvDestroyWindow( "Calibration" );
  
}  



  ////////////////////////////////////////////
  // Find the two laser lines on the corner //
  ////////////////////////////////////////////
  
  std::cout << std::endl;
  std::cout << " 2) Detecting laser line." << std::endl;
  std::cout << "    Please adjust brightness and Hough parameter to fit the laser." << std::endl;
  std::cout << "    Then press space bar to continue." << std::endl;
  
  cvNamedWindow( "Laser detection" );
  cv::createTrackbar("Threshold Value", "Laser detection", &V_cut, 250, onTrackbar);
  onTrackbar(0);
  cv::createTrackbar("Threshold lines", "Laser detection", &H_cut, 200, onTrackbar);
  onTrackbar(0);
  
  std::vector<cv::Vec2f> lines; // contains the set of lines found by HoughLines
  cv::Point ptX;
  cv::Point pt0;
  cv::Point ptZ;
  
  double aVert = 0;
  double bVert = 0;
  double aDiag = 0;
  double bDiag = 0;
  
  // Type space when lines are recognized
  while(key != ' ')
  {
    cv::cvtColor(image, HSV_image, CV_BGR2HSV);
    
    // Threshold HSV_image with V_cut
    for(int i=0; i<HSV_image.rows; i++)
    {
      for(int j=0; j<HSV_image.cols; j++)
      {
        if(HSV_image.at<cv::Vec3b>(i,j)[2] < V_cut)
        {
          HSV_image.at<cv::Vec3b>(i,j)[0] = 0;
          HSV_image.at<cv::Vec3b>(i,j)[1] = 0;
          HSV_image.at<cv::Vec3b>(i,j)[2] = 0;
        }
        else
        {
          HSV_image.at<cv::Vec3b>(i,j) = HSV_image.at<cv::Vec3b>(i,j);
        }
      }
    }
    cv::blur( HSV_image, HSV_image, cv::Size(3,3) );

    cvtColor( HSV_image, HSV_gray_image, CV_BGR2GRAY );
    
    imshow("Laser detection", HSV_gray_image);
  
    // Find middle of laser
    HSV_gray_image2 = HSV_gray_image.clone();
    HSV_gray_image2 = cv::Scalar(0);
    std::vector<int> row;
    for(int i=0; i<HSV_gray_image.rows; i++)
    {
      for(int j=0; j<HSV_gray_image.cols; j++)
      {
        if(HSV_gray_image.at<unsigned char>(i,j) != 0)
          row.push_back(j);
      }
//        std::cerr << row.size() << std::endl;
      if(row.size()!=0)
        HSV_gray_image2.at<unsigned char>(i,row[row.size()/2]) = 255;
      row.clear();
    }
    
    
    
    
    
    // Find lines with HoughLines function
    capture >> image;
    imageWL = image.clone();
    HoughLines( HSV_gray_image2, lines, 1, CV_PI/180, H_cut );
    
    // Collect the pairs of points defining the lines
    std::vector< std::pair<cv::Point, cv::Point> > setPts;
    for( size_t i = 0; i < lines.size(); i++ )
    {
      float rho = lines[i][0];
      float theta = lines[i][1];
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      cv::Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
      cv::Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));
      std::pair<cv::Point, cv::Point> pair;
      pair.first = pt1;
      pair.second = pt2;
//        std::cerr << pt1 << " " << pt2 << std::endl;
      setPts.push_back(pair);
//        line( imgWL, pt1, pt2, Scalar(0,0,255), 1, CV_AA );
    }

    // Separate the two sets of lines according to their slope
    std::vector< std::pair< cv::Point, cv::Point> > vertLines;
    std::vector< std::pair< cv::Point, cv::Point> > diagLines;
    for(size_t i = 0;i<setPts.size();++i)
    {
      if((setPts[i].first.x-setPts[i].second.x) != 0)
      {
        if(fabs((setPts[i].first.y-setPts[i].second.y)/(setPts[i].first.x-setPts[i].second.x)) > 30 ) // 30 is arbitrary
        {
          vertLines.push_back(setPts[i]);
        }
        else
        {
          diagLines.push_back(setPts[i]);
        }
      }
      else
      {
        vertLines.push_back(setPts[i]);
      }
    }

    // Average the two sets of lines to get 2 lines
    // (also check that lines are defined by points in the same direction)
    
    double avgVertX1=0;
    double avgVertY1=0;
    double avgVertX2=0;
    double avgVertY2=0;
    double avgDiagX1=0;
    double avgDiagY1=0;
    double avgDiagX2=0;
    double avgDiagY2=0;
    
    for(size_t i = 0;i<vertLines.size();++i)
    {
      
      if(vertLines[i].first.y < vertLines[i].second.y)
      {
        avgVertX1 += vertLines[i].first.x;
        avgVertY1 += vertLines[i].first.y;
        avgVertX2 += vertLines[i].second.x;
        avgVertY2 += vertLines[i].second.y;
      }
      else
      {
        avgVertX1 += vertLines[i].second.x;
        avgVertY1 += vertLines[i].second.y;
        avgVertX2 += vertLines[i].second.x;
        avgVertY2 += vertLines[i].second.y;
      }
    }
    avgVertX1 /= vertLines.size();
    avgVertY1 /= vertLines.size();
    avgVertX2 /= vertLines.size();
    avgVertY2 /= vertLines.size();
    
    cv::Point ptVert1(avgVertX1,avgVertY1);
    cv::Point ptVert2(avgVertX2,avgVertY2);
    
    avgVertX1=0;
    avgVertY1=0;
    avgVertX2=0;
    avgVertY2=0;
    avgDiagX1=0;
    avgDiagY1=0;
    avgDiagX2=0;
    avgDiagY2=0;
    
    for(size_t i = 0;i<diagLines.size();++i)
    {
      
      if(diagLines[i].first.y < diagLines[i].second.y)
      {
        avgDiagX1 += diagLines[i].first.x;
        avgDiagY1 += diagLines[i].first.y;
        avgDiagX2 += diagLines[i].second.x;
        avgDiagY2 += diagLines[i].second.y;
      }
      else
      {
        avgDiagX1 += diagLines[i].second.x;
        avgDiagY1 += diagLines[i].second.y;
        avgDiagX2 += diagLines[i].second.x;
        avgDiagY2 += diagLines[i].second.y;
      }
    }
    avgDiagX1 /= diagLines.size();
    avgDiagY1 /= diagLines.size();
    avgDiagX2 /= diagLines.size();
    avgDiagY2 /= diagLines.size();
    
    cv::Point ptDiag1(avgDiagX1,avgDiagY1);
    cv::Point ptDiag2(avgDiagX2,avgDiagY2);
    
//    std::cerr << "ptVert1 " << ptVert1 << "; ptVert2" << ptVert2 << std::endl;
//    std::cerr << "ptDiag1 " << ptDiag1 << "; ptDiag2" << ptDiag2 << std::endl;
    
    
    // Cut lines according to image dimensions. Equations: ax+b=y

    if (ptVert1.x==ptVert2.x)
      aVert=999999999999;
    else
      aVert = ((double)ptVert1.y-(double)ptVert2.y)/((double)ptVert1.x-(double)ptVert2.x);
    //double bVert = (double)ptVert1.y-(double)aVert*(double)ptVert1.x;
    bVert = (double)ptVert1.y-(double)aVert*(double)ptVert1.x;
    
//      std::cerr << "a,b vert " << aVert << " " << bVert << std::endl;
    
//    double aDiag = ((double)ptDiag1.y-(double)ptDiag2.y)/((double)ptDiag1.x-(double)ptDiag2.x);
//    double bDiag = (double)ptDiag1.y-(double)aDiag*(double)ptDiag1.x;
    aDiag = ((double)ptDiag1.y-(double)ptDiag2.y)/((double)ptDiag1.x-(double)ptDiag2.x);
    bDiag = (double)ptDiag1.y-(double)aDiag*(double)ptDiag1.x;

//      std::cerr << std::endl << "a,b vert " << aVert << " " << bVert << std::endl;    
//      std::cerr << "a,b diag " << aDiag << " " << bDiag << std::endl;
//      
      
      
    
    // Coordinates of the intersection of the two lines
    double xInter= -(bVert-bDiag)/(aVert-aDiag);
    double yInter= aDiag*xInter + bDiag;
    
//    std::cerr << "Pinter " << xInter << " " << yInter << std::endl;
    
    cv::Point ptInter(xInter, yInter);
    
    
    
    
//      std::cerr << ptInter << std::endl;

    // /!\ WARNING : DIAG MUST CROSS THE TOP OF THE IMAGE      
    ptVert1.y = 0;
    if(aVert < 9999)
      ptVert1.x = -bVert/aVert;
    else
      ptVert1.x = xInter;
    ptVert2 = ptInter;
    
    ptDiag1.y = image.rows;
    ptDiag1.x = (image.rows-bDiag)/aDiag;
    ptDiag2 = ptInter;
    
//      std::cerr << ptVert1 << " " << ptVert2 << std::endl;
//      std::cerr << ptDiag1 << " " << ptDiag2 << std::endl;
//      std::cerr << "--------------" << std::endl;
//            
    line( imageWL, ptVert1, ptVert2, cv::Scalar(0,0,255), 1, CV_AA );
    line( imageWL, ptDiag1, ptDiag2, cv::Scalar(0,0,255), 1, CV_AA );
    
    //line( imgWL, ptInter, adel, Scalar(255,0,0), 1, CV_AA );
    
    
      //c = cvWaitKey( 15 );
    //}
  
    
  
	  ptX=ptDiag1;
	  pt0=ptInter;
	  ptZ=ptVert1;
	  
//    std::cerr << ptZ << " " << pt0 << " " << ptX << std::endl << "------------";
	  
	  circle(imageWL,ptX,10,cv::Scalar(0,255,0),3);
	  circle(imageWL,pt0,10,cv::Scalar(0,255,0),3);
	  circle(imageWL,ptZ,10,cv::Scalar(0,255,0),3);

    lines.clear();
    
    
    key = cv::waitKey(10);
    imshow("Detected laser", imageWL);
    imshow("Laser detection", HSV_gray_image2);
    capture >> image;
  }

  cvDestroyWindow("Detected laser");
	cvDestroyWindow("Laser detection");
	
	
//  std::cerr << ptZ << " " << pt0 << " " << ptX << std::endl;





  
  ////////////////////////////////////////////////
  // Find the normal n vector to the laser plan //
  ////////////////////////////////////////////////
  
  std::cout << std::endl;
  std::cout << " 3) Computing the normal vector of the laser plane." << std::endl;
  
  // Real points
  cv::Point3d rZ(0,0,30.5);
  cv::Point3d r0(0,0,0);
  cv::Point3d rX(37.3,0,0);

  // Z and X steps	
	double stepZ = fabs(rZ.z-r0.z) / fabs((double)pt0.y-(double)ptZ.y);
	double stepX = fabs(rX.x-r0.x) / fabs((double)ptX.y-(double)pt0.y);
	
	
//	std::vector< std::vector<double> > A;
	cv::Mat A(3*image.rows,3,cv::DataType<double>::type);
//	std::vector<double> B;
	cv::Mat B(3*image.rows,3,cv::DataType<double>::type);
//	std::vector<double> r;
	double u=0;
  int cpt=0;

  cv::Point3d realP(0,0,0);

	// Along the vertical line
	for(double v=(double)ptZ.y; v<(double)pt0.y; ++v)
	{
	  if(aVert > 9999)
	    u=(double)ptZ.x; // computer's infinity
	  else
	    u=((double)v-bVert)/aVert;
//	  std::cerr << "u,v " << u << " " << v <<  "----- Z  " << (rZz-cpt*stepZ)  << std::endl;
    
    realP = cv::Point3d(0,0,rZ.z-cpt*stepZ);
    
//    
//    r.push_back(u*realP.x); r.push_back(v*realP.x); r.push_back(realP.x);
//    A.push_back(r);
//    r.clear();
//    
//    r.push_back(u*realP.y); r.push_back(v*realP.y); r.push_back(realP.y);
//    A.push_back(r);
//    r.clear();
//    
//    r.push_back(u*realP.z); r.push_back(v*realP.z); r.push_back(realP.z);
//    A.push_back(r);
//    r.clear();
//    
//    B.push_back(u);
//    B.push_back(v);
//    B.push_back(1);
    A.at<double>(3*v,0)   = u*realP.x ; A.at<double>(3*v,1)   = v*realP.x ; A.at<double>(3*v,2)   = realP.x;
    A.at<double>(3*v+1,0) = u*realP.y ; A.at<double>(3*v+1,1) = v*realP.y ; A.at<double>(3*v+1,2) = realP.y;
    A.at<double>(3*v+2,0) = u*realP.z ; A.at<double>(3*v+2,1) = v*realP.z ; A.at<double>(3*v+2,2) = realP.z;
    B.at<double>(3*v,0) = u;
    B.at<double>(3*v+1,0) = v;
    B.at<double>(3*v+2,0) = 1;
	  ++cpt;
	}

//  ///// ADEL
//  r.push_back(-1); r.push_back(-1); r.push_back(-1);
//  A.push_back(r);
//  r.clear();
//  ////// FIN ADEL

//  std::cerr << "stepX stepZ" << stepX << " " << stepZ << std::endl;

  cpt=0;
//  r.clear();
	// Along the diagonal line
	for(double v=(double)pt0.y; v<(double)ptX.y; ++v)
	{
	  u=(v-bDiag)/aDiag;
//	  std::cerr << "u,v  " << u << "," << v << std::endl;
	  realP = cv::Point3d(r0.x+cpt*stepX,0,0); // moves along X axis, from the origin to rX
//	  std::cerr << realP << std::endl;
	  
//	  r.push_back(u*realP.x); r.push_back(v*realP.x); r.push_back(realP.x);
//    A.push_back(r);
//    r.clear();
//    
//    r.push_back(u*realP.y); r.push_back(v*realP.y); r.push_back(realP.y);
//    A.push_back(r);
//    r.clear();
//    
//    r.push_back(u*realP.z); r.push_back(v*realP.z); r.push_back(realP.z);
//    A.push_back(r);
//    r.clear();
//    
//    B.push_back(u);
//    B.push_back(v);
//    B.push_back(1);
//    A.at<double>(3*v,0) = u*(r0x+cpt*stepX) ; A.at<double>(3*v,1) = v*(r0x+cpt*stepX) ; A.at<double>(3*v,2) = (r0x+cpt*stepX);
//    A.at<double>(3*v+1,0) = 0 ; A.at<double>(3*v+1,1) = 0 ; A.at<double>(3*v+1,2) = 0;
//    A.at<double>(3*v+2,0) = 0 ; A.at<double>(3*v+2,1) = 0 ; A.at<double>(3*v+2,2) = 0;
//    B.at<double>(3*v,0) = u;
//    B.at<double>(3*v+1,0) = v;
//    B.at<double>(3*v+2,0) = 1;
    A.at<double>(3*v,0)   = u*realP.x ; A.at<double>(3*v,1)   = v*realP.x ; A.at<double>(3*v,2)   = realP.x;
    A.at<double>(3*v+1,0) = u*realP.y ; A.at<double>(3*v+1,1) = v*realP.y ; A.at<double>(3*v+1,2) = realP.y;
    A.at<double>(3*v+2,0) = u*realP.z ; A.at<double>(3*v+2,1) = v*realP.z ; A.at<double>(3*v+2,2) = realP.z;
    B.at<double>(3*v,0) = u;
    B.at<double>(3*v+1,0) = v;
    B.at<double>(3*v+2,0) = 1;


    ++cpt;
	}
  
//  std::cerr << A.rows << "x" << A.cols << std::endl;
//  for(size_t i=0;i<A.size();++i)
//	{
//	  for(size_t j=0;j<A[i].size()-1;++j)
//	  {
//	    std::cout << A[i][j] << " ";
//	  }
//	  std::cout << A[i][A[i].size()-1] << ";" << std::endl;
//	}
//	std::cout << "---" << std::endl;
//	
//	for(size_t i=0;i<B.size();++i)
//	{
//    //std::cout << B[i] << std::endl;
//  }

  
  cv::Mat N = ((A.t()*A).inv()*(A.t()*B)).col(0);
//  std::cout << "Normal found : " << std::endl << N << std::endl;
  std::cout << "    Saving normal vector." << std::endl;
  fs << "normal" << N;  
  fs.release();
  
  std::cout << std::endl;
  std::cout << " Done! Parameters are stored in the file \"" << ss.str() << "\"." << std::endl << std::endl;

  return 0;
}



