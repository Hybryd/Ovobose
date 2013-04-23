#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <cv.h>
#include <highgui.h>

using namespace cv;

int V_cut=200;
int H_cut=100;

std::vector< std::vector <double> > A;

std::vector <double> image2Real(Point p)
{
  std::vector <double> res;
  double s = A[3][0]*(double)p.x + A[3][1]*(double)p.y + A[3][2];
  std::cerr << "s=" << s << std::endl;
  double X=0;
  double Y=0;
  double Z=0;
  s=100000;
  if(s!=0)
  {
    X = (A[0][0]*(double)p.x + A[0][1]*(double)p.y + A[0][2])/s;
    Y = (A[1][0]*(double)p.x + A[1][1]*(double)p.y + A[1][2])/s;
    Z = (A[2][0]*(double)p.x + A[2][1]*(double)p.y + A[2][2])/s;
    res.push_back(X);
    res.push_back(Y);
    res.push_back(Z);
  }
  else
    std::cerr << "Error in image2Real : s=0" << std::endl;

  return res;
}


using namespace cv;
int main(int argc, char ** argv)
{

  // Transformation matrix (svd octave function)
  std::vector< double > row;
//  row.push_back(1.81094195512617e+05);
//  row.push_back(5.20207557518076e+04);
//  row.push_back(9.04125850844210e+03);
//  A.push_back(row);
//  row.clear();
//  
//  row.push_back(8.25574683061401e+03);
//  row.push_back(6.73559969808529e+03);
//  row.push_back(2.08334309012028e+03);
//  A.push_back(row);
//  row.clear();
//  
//  row.push_back(1.65703688685031e+03);
//  row.push_back(5.70239312337582e+02);
//  row.push_back(1.04750597814946e+01);
//  A.push_back(row);
//  row.clear(); 
//   
//  row.push_back(1.19527857653320e+00);
//  row.push_back(7.58740131749232e-01);
//  row.push_back(2.98704060231241e-02);
//  A.push_back(row);
//  row.clear(); 
   
  row.push_back(3.40029568434935e+05);
  row.push_back(9.62738363555907e+04);
  row.push_back(1.74323335505271e+04);
  A.push_back(row);
  row.clear();
  row.push_back(1.53981017646603e+04);
  row.push_back(1.36045385557371e+04);
  row.push_back(3.83337816651352e+03);
  A.push_back(row);
  row.clear();
  row.push_back(2.94214152092455e+03);
  row.push_back(1.10431653825115e+03);
  row.push_back(9.03764719532437e+00);
  A.push_back(row);
  row.clear();
  row.push_back(9.92574144929398e-01);
  row.push_back(6.04487556479935e-01);
  row.push_back(2.14714106616099e-02);
  A.push_back(row);
  row.clear();
   
   
  

  
//////  CvCapture * capture = NULL; // capture
//////  IplImage * image = NULL; // capture assignation
  cv::Mat image = imread("pics/corner.jpg");
  cv::Mat image2; // clone of image
  cv::Mat image2HSV; // clone of image HSV version
  cv::Mat image2HSV2; // clone of image2HSV
  cv::Mat image2HSVGRAY; // clone of image2HSV
  
  cv::Mat img; // image for the calibration
  cv::Mat imgGRAY; // image for the calibration GRAY
  cv::Mat imgGRAY2; // image for the calibration GRAY
  cv::Mat imgWL; // image withlines

//////  capture = cvCreateCameraCapture( 0 );
//////	if(capture < 0)
//////	{
//////	  std::cerr << "Can not initialize video capture." << std::endl;
//////	  return 0;
//////  }
	cvNamedWindow( "Real image" );
	cvNamedWindow( "Thresholded GRAY" );
//////	image = cvQueryFrame( capture );
	
	int c=0;
	int cpt=0;
//////	while( image && c != 32  && cpt==0) //spacebar to take image
  while(c != 32  && cpt==0) //spacebar to take image
	{
	  image2 = image;
	  cv::cvtColor(image2, image2HSV, CV_BGR2HSV);
	  image2HSV2 = image2HSV;
	  
	  
//////	  image = cvQueryFrame( capture ); 
	  
	  
	  cv::cvtColor(image2, image2HSV, CV_BGR2HSV);
	  image2HSV2 = image2HSV;
	  
	  // threshold image2HSV2
	  for(int i=0; i<image2HSV2.rows; i++)
    {
      for(int j=0; j<image2HSV2.cols; j++)
      {
        if(image2HSV.at<cv::Vec3b>(i,j)[2] < V_cut)
        {
          image2HSV2.at<cv::Vec3b>(i,j)[0] = 0;
          image2HSV2.at<cv::Vec3b>(i,j)[1] = 0;
          image2HSV2.at<cv::Vec3b>(i,j)[2] = 0;
        }
        else
        {
          image2HSV2.at<cv::Vec3b>(i,j) = image2HSV.at<cv::Vec3b>(i,j);
        }
      }
    }
    cv::blur( image2HSV2, image2HSV2, cv::Size(3,3) );
	  cvtColor( image2HSV2, image2HSVGRAY, CV_BGR2GRAY );
	  
    imshow( "Real image", image2 );
	  cv::putText(image2, "Press 'Space' to take the snapshot", cvPoint(200,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.45, cvScalar(200,200,200), 1);
	  
	  c = cvWaitKey( 15 );
	  
    img = image2;
    imgGRAY = image2HSVGRAY;
	  
	  
  
    // Find middle of laser
    imgGRAY2 = imgGRAY.clone();
    imgGRAY2 = Scalar(0);
    std::vector<int> row;
    for(int i=0; i<imgGRAY.rows; i++)
    {
      for(int j=0; j<imgGRAY.cols; j++)
      {
        if(imgGRAY.at<unsigned char>(i,j) != 0)
          row.push_back(j);
      }
      if(row.size()!=0)
      {
        imgGRAY2.at<unsigned char>(i,row[row.size()/2]) = 255;
//        Point p(i,row[row.size()/2]);
//        std::vector<double> real = image2Real(p);
//        std::cout << real[0] << " " << real[1]  << " " << real[2] << std::endl;
      }
      row.clear();
    }
    
    for(int i=0; i<imgGRAY2.rows; i++)
    {
      for(int j=0; j<imgGRAY2.cols; j++)
      {
        if(imgGRAY2.at<unsigned char>(i,j)==255)
        {
          Point p(i,row[row.size()/2]);
          std::vector<double> real = image2Real(p);
          std::cout << real[0] << " " << real[1]  << " " << real[2] << std::endl;
        }
      }
    }
    
    
    imshow("Thresholded GRAY", imgGRAY2);
    ++cpt;
  }

  return 0;
}
