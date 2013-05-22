#include "Calibration.h"
#include "PostProcessor.h"
#include "Scan.h"
#include <stdio.h>
#include <stdlib.h>
//#include <iostream>
//#include <sstream>


int main(int argc, char ** argv )
{
  std::string cal="";
  if(argc>1)
  {
    cal=std::string(argv[1]);
  }
  else
//  {
//    std::cerr << "Usage : ./main [\"cal\"]" << std::endl << "\"cal\" option is for calibration." << std::endl;
//    return -1;
//  }
  
  if(cal=="cal")
  {
    Calibration cal("parameters.xml");
    cal.circularPattern(10,5.6);
    cal.save("matM","vecN");
  }
  
  std::cout << "Scanning..." << std::endl;
  Scan scan("parameters.xml","out.gp",200);
  scan.read("matM","vecN");
  scan.launch();
  scan.save();
  
  std::cout << "Post processing..." << std::endl;
  PostProcessor p(scan.getData(), "out_PP.gp");
  p.keepInCylinder(cv::Point3d(0,0,0),1,100);
  p.save();
  
  
  
  return 0;
}
