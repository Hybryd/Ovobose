#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////
// Read parameters from a file and scan the laser line to compute //
// 3D corresponding coordinates.                                  //
// The output file is aimed to be read by gnuplot                 //
//                                                                //
// Usage: ./scan [parameterFile.xml] [output.gp]                  //
////////////////////////////////////////////////////////////////////

// Values for the manual thresholding of images during the laser detection
int V_cut=200;

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
  cv::Mat HSV_image;
  cv::Mat HSV_gray_image;
  cv::Mat HSV_gray_image2;

  cv::Mat matM(3,4,cv::DataType<double>::type);
  cv::Mat normal(4,1,cv::DataType<double>::type);
  
  cv::Mat finalVec(3,1,cv::DataType<double>::type);
  cv::Mat finalMat(3,3,cv::DataType<double>::type);
  
  std::stringstream ss;
  
  if(argc > 1)
    ss << argv[1];
  else
    ss << "param.xml";
  
  cv::FileStorage fs(ss.str(), cv::FileStorage::READ); // parameters file
  fs["matM"] >> matM;
  fs["normal"] >> normal;
  
  cv::VideoCapture capture = cv::VideoCapture(0);
  if(!capture.isOpened())
  {
    std::cerr << "Unable to open capture video." << std::endl;
    return -1;
  }
  capture >> image;
  
  
  cvNamedWindow( "Scan" );
  cv::createTrackbar("Brightness", "Scan", &V_cut, 250, onTrackbar);
  onTrackbar(0);
  
  // Type space when ready to scan
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
    

    // Find the middle of the laser line
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
      if(row.size()!=0)
        HSV_gray_image2.at<unsigned char>(i,row[row.size()/2]) = 255;
      row.clear();
    }
    
    imshow("Scan", HSV_gray_image2);
    key = cv::waitKey(10);
    capture >> image;
  }
  
  
  
  // Find the pixels and compute corresponding 3D point
  // (u=j v=i)
  
  std::vector<cv::Mat> L; // will contain the real points
  for(int v=0; v<HSV_gray_image2.rows; v++)
  {
    for(int u=0; u<HSV_gray_image2.cols; u++)
    {
      if(HSV_gray_image2.at<unsigned char>(v,u) == 255) // enlighted pixel
      {

        finalMat.at<double>(0,0) = matM.at<double>(0,0) - matM.at<double>(2,0)*u;
        finalMat.at<double>(0,1) = matM.at<double>(0,1) - matM.at<double>(2,1)*u;
        finalMat.at<double>(0,2) = matM.at<double>(0,2) - matM.at<double>(2,2)*u;

        finalMat.at<double>(1,0) = matM.at<double>(1,0) - matM.at<double>(2,0)*v;
        finalMat.at<double>(1,1) = matM.at<double>(1,1) - matM.at<double>(2,1)*v;
        finalMat.at<double>(1,2) = matM.at<double>(1,2) - matM.at<double>(2,2)*v;
        
        finalMat.at<double>(2,0) = normal.at<double>(0,0);
        finalMat.at<double>(2,1) = normal.at<double>(0,1);
        finalMat.at<double>(2,2) = normal.at<double>(0,2);
        
        finalVec.at<double>(0,0) = matM.at<double>(2,3)*u - matM.at<double>(0,3);
        finalVec.at<double>(0,1) = matM.at<double>(2,3)*v - matM.at<double>(1,3);
        finalVec.at<double>(0,2) = -normal.at<double>(0,3);

        finalMat=finalMat.inv();
        
        L.push_back(finalMat*finalVec);

      }
    }
  }
  cvDestroyWindow("Scan");
  
  // Save data
  ss.str("");
  if(argc > 2)
    ss << argv[2];
  else
    ss << "output.gp";
  
//  std::ofstream out;
//  out.open(ss.str());
  std::ofstream out(ss.str().c_str());
  if(!out.is_open())
  {
    std::cerr << "Error: unable to open " << ss.str() << std::endl;
    return -1;
  }
  
  for(size_t i=0;i<L.size();++i)
  {
    out << L[i].at<double>(0,0) << " " << L[i].at<double>(0,1) << " " << L[i].at<double>(0,2) << std::endl;
  }
  //std::cerr << L.size()<< std::endl;

  return 0;
}




