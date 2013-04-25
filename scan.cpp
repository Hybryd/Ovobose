#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

// Values for the manual thresholding of images during the laser detection
int V_cut=200;
int H_cut=90;


// Function called by the trackbar
void onTrackbar(int, void* = NULL)
{
}


// Function pi that changes the pixel in homogeneous coordinates [u,v,1] to the point [su,sv,s] that lies on the laser plane
cv::Mat pi(cv::Point3d & p, cv::Mat & n)
{
  cv::Mat res=cv::Mat::zeros(3,1,cv::DataType<double>::type);
  double lambda = 1.;///(p.x*n.at<double>(0,0) + p.y*n.at<double>(0,1) + n.at<double>(0,2)); // WHY DOES IT WORK ???
  res.at<double>(0,0) = lambda*p.x;
  res.at<double>(0,1) = lambda*p.y;
  res.at<double>(0,2) = lambda;
  return res;
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
  cv::Mat intrinsic;
  cv::Mat distorsion; // not used here
  cv::Mat rotation;

  cv::Mat translation;
  cv::Mat RT(3,4,cv::DataType<double>::type);
  cv::Mat prod(3,4,cv::DataType<double>::type);
  cv::Mat prodMat(3,3,cv::DataType<double>::type);
  cv::Mat prodMatInv(3,3,cv::DataType<double>::type);
  cv::Mat prodVec(3,1,cv::DataType<double>::type);
  cv::Mat normal(3,1,cv::DataType<double>::type);
  
  std::stringstream ss;
  
  if(argc > 1)
    ss << argv[1];
  else
    ss << "param.xml";
  cv::FileStorage fs(ss.str(), cv::FileStorage::READ); // parameters file

  // Read parameters  
  std::cout << " Read parameters stored in \"" << ss.str() << "\"." << std::endl;
  fs["intrinsic"] >> intrinsic;
  fs["distorsion"] >> distorsion;
  fs["rotation"] >> rotation;
  fs["translation"] >> translation;
  fs["normal"] >> normal;
  fs["V_cut"] >> V_cut;
  fs["H_cut"] >> H_cut;
  
  fs.release();
  
//  std::cout << "Intrinsic matrix : " << std::endl << intrinsic << std::endl;
//  std::cout << "Distortion matrix : " << std::endl << distorsion << std::endl;
//  std::cout << "Rotation matrix : " << std::endl << rotation << std::endl;
//  std::cout << "Translation vector : " << std::endl << translation << std::endl;
//  std::cout << "Normal vector : " << std::endl << normal << std::endl;
  
  std::cerr << "koko" << std::endl;
  
  // Concatenate [R|T]
  for(int i=0;i<rotation.rows;++i)
  {
    for(int j=0;j<rotation.cols;++j)
    {
      RT.at<double>(i,j) = rotation.at<double>(i,j);
    }
    RT.at<double>(i,3) = translation.at<double>(i,0);
  }
  
//  std::cerr << "koko2" << prod.rows << "x" << prod.cols << std::endl;
  
  // Compute prod = intrinsic.[R|T]
  prod = intrinsic*RT;
//  std::cerr << "prod" << prod << std::endl;
  // Decompose prod in [prodMat|prodVec]
 // prodMat = prod(cv::Range(1, 1), cv::Range(3, 3));
  for(int i=0;i<3;++i)
  {
    for(int j=0;j<3;++j)
    {
      prodMat.at<double>(i,j) = prod.at<double>(i,j);
    }
    prodVec.at<double>(i,0) = prod.at<double>(i,3);
  }
//  std::cerr << "prodMat" << prodMat << std::endl;
//  std::cerr << "prodVec" << prodVec << std::endl;
//   Inverse of prodMat
  prodMatInv = prodMat.inv();
  
  
 //  Given a pixel [u,v] on the red line, to get the 3D point [X,Y,Z] that lies on the laser plane, apply the formula:
 //  [X,Y,Z] = prodMatInv . ( pi([u,v,1]) - prodVec )
  
  
  
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
  
  // Type space when ready
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
    
    imshow("Scan", HSV_gray_image);



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
    key = cv::waitKey(10);
    capture >> image;
    
  }
  
  // Find the pixels and compute corresponding 3D point
  std::vector<cv::Mat> L;// = cv::Mat::zeros(image.rows,1,cv::DataType<double>::type);
  for(int i=0; i<HSV_gray_image2.rows; i++)
  {
    for(int j=0; j<HSV_gray_image2.cols; j++)
    {
      if(HSV_gray_image2.at<unsigned char>(i,j) == 255) // enlighted pixel
      {
        cv::Point3d p(i,j,1);
        cv::Mat P=prodMatInv*( pi(p,normal) - prodVec );
        L.push_back(P);
      }
    }
  }
  cvDestroyWindow("Scan");
  
  // prints
//  for(int i=0;i<L.size();++i)
//  {
//    std::cout << L[i].at<double>(0,0) << " " << L[i].at<double>(0,1) << " " << L[i].at<double>(0,2) << std::endl;
//  }
//  std::cerr << L.size()<< std::endl;

  return 0;
}




