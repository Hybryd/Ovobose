#include "Scan.h"


Scan::Scan()
{
  Vcut=200;
  paramFile="param.xml";
  outputFile="output.gp";
  cvNamedWindow( "Scan" );
}

Scan::Scan(std::string pFile, std::string outFile, int pVcut)
{
  if (pFile.size() > 4 && pFile.find(".xml")==pFile.size()-4)
  {
    paramFile = pFile;
  }
  else
  {
    std::cerr << "Warning: bad name for parameter file. Parameters are read from \"param.xml\"." << std::endl;
    paramFile = "param.xml";
  }
  
  if (outFile.size() > 3 && outFile.find(".gp")==outFile.size()-3)
  {
    outputFile = outFile;
  }
  else
  {
    std::cerr << "Warning: bad name for output file. Data will be stored in \"output.gp\"." << std::endl;
    paramFile = "output.gp";
  }

  if(0 < pVcut && pVcut < 250)
    Vcut=pVcut;
  else
    Vcut=200;
    
  cvNamedWindow( "Scan" );
}

void Scan::read(std::string pNameMatrix, std::string pNameNormal)
{
  std::string lM = pNameMatrix;
  std::string lN = pNameNormal;
  
  if(pNameMatrix.size()==0 || pNameNormal.size()==0 || pNameMatrix.find(" ") != std::string::npos || pNameNormal.find(" ") != std::string::npos)
  {
    std::cerr << "Warning: bad name for data. The transformation matrix will be saved as \"matM\" and the normal vector as \"vecN\"." << std::endl;
    lM = "matM";
    lN = "vecN";
  }
  cv::FileStorage fs(paramFile, cv::FileStorage::READ); // parameters file
  
  matM.create(3,4,cv::DataType<double>::type);
  vecN.create(4,1,cv::DataType<double>::type);
  
  fs[lM] >> matM;
  fs[lN] >> vecN;
}

// Function called by the trackbar
void onTrackbar(int, void* = NULL)
{
}

void Scan::launch()
{
  int key=0;
  cv::Mat image;
  cv::Mat HSV_image;
  cv::Mat HSV_gray_image;
  cv::Mat HSV_gray_image2;

  cv::VideoCapture capture = cv::VideoCapture(0);
  if(!capture.isOpened())
  {
    std::cerr << "Error: unable to open capture video." << std::endl;
    exit(-1);
  }
  capture >> image;
  
  cv::createTrackbar("Brightness", "Scan", &Vcut, 250, onTrackbar);
  onTrackbar(0);
  
  // Type 'q' to end scanning
  while(key != 'q')
  {
    cv::cvtColor(image, HSV_image, CV_BGR2HSV);
    
    // Threshold HSV_image with V_cut
    for(int i=0; i<HSV_image.rows; i++)
    {
      for(int j=0; j<HSV_image.cols; j++)
      {
        if(HSV_image.at<cv::Vec3b>(i,j)[2] < Vcut)
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
    
    
    
    // Type space when ready to scan
    if(key==' ')
    {
      measure(HSV_gray_image2);
    }
    
    
    
    imshow("Scan", HSV_gray_image2);
    key = cv::waitKey(10);
    capture >> image;
  }
}



void Scan::measure(cv::Mat & current)
{
  cv::Mat finalVec(3,1,cv::DataType<double>::type);
  cv::Mat finalMat(3,3,cv::DataType<double>::type);

  // Find the pixels and compute corresponding 3D point
  // (u=j v=i)
  
  std::vector<cv::Mat> L; // will contain the real points
  for(int v=0; v<current.rows; v++)
  {
    for(int u=0; u<current.cols; u++)
    {
      if(current.at<unsigned char>(v,u) == 255) // enlighted pixel
      {

        finalMat.at<double>(0,0) = matM.at<double>(0,0) - matM.at<double>(2,0)*u;
        finalMat.at<double>(0,1) = matM.at<double>(0,1) - matM.at<double>(2,1)*u;
        finalMat.at<double>(0,2) = matM.at<double>(0,2) - matM.at<double>(2,2)*u;

        finalMat.at<double>(1,0) = matM.at<double>(1,0) - matM.at<double>(2,0)*v;
        finalMat.at<double>(1,1) = matM.at<double>(1,1) - matM.at<double>(2,1)*v;
        finalMat.at<double>(1,2) = matM.at<double>(1,2) - matM.at<double>(2,2)*v;
        
        finalMat.at<double>(2,0) = vecN.at<double>(0,0);
        finalMat.at<double>(2,1) = vecN.at<double>(0,1);
        finalMat.at<double>(2,2) = vecN.at<double>(0,2);
        
        finalVec.at<double>(0,0) = matM.at<double>(2,3)*u - matM.at<double>(0,3);
        finalVec.at<double>(0,1) = matM.at<double>(2,3)*v - matM.at<double>(1,3);
        finalVec.at<double>(0,2) = -vecN.at<double>(0,3);

        finalMat=finalMat.inv();
        
        L.push_back(finalMat*finalVec);

      }
    }
  }
  data.push_back(L);
}



// Save according to the gnuplot format:
// X1 Y1 Z1
// X2 Y2 Z2
// :  :  :

void Scan::save()
{
  std::ofstream out(outputFile.c_str());
  if(!out.is_open())
  {
    std::cerr << "Error: unable to open " << outputFile << std::endl;
    exit(-1);
  }
  
  for(size_t i=0;i<data.size();++i)
  {
    for(size_t j=0;j<data[i].size();++j)
    {
      out << data[i][j].at<double>(0,0) << " " << data[i][j].at<double>(0,1) << " " << data[i][j].at<double>(0,2) << std::endl;
    }
  }
}




