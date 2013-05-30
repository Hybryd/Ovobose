#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>


int main(int argc, char ** argv)
{
  int key=0;
  cv::VideoCapture capture = cv::VideoCapture(0);
  cv::Mat image;
  if(!capture.isOpened())
  {
    std::cerr << "ERROR: Unable to open capture video" << std::endl;
    return -1;
  }
  capture >> image;
  
  while(key!='q')
  {
    key = cv::waitKey(10);
    imshow("Just see.", image);
    capture >> image;
  }
  
  
  return 0;
}
