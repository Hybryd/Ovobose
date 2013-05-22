#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <cv.h>
#include <fstream>
#include <iostream>
#include <string>


class PostProcessor
{
protected:
  std::string                             inputFile;  // contains raw data
  std::string                             outputFile; // contains processed data
  std::vector<cv::Mat>                    data;       // raw data
  std::vector<cv::Mat>                    dataPP;     // processed data
  


public:
  PostProcessor();
  PostProcessor(std::string inFile, std::string outFile);
  PostProcessor(std::vector<cv::Mat> & pData, std::string outFile);
  
  void read();
  void keepInCylinder(cv::Point3d center, double radius, double height);
  void save();

};

#endif
