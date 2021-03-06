#include <stdio.h>
#include <stdlib.h>
//#include <iostream>
//#include <sstream>

#include "../Calibration.h"
#include "../DataHandler.h"
#include "../Scan.h"


// Usage ./main_scan [scan.param]

int main(int argc, char ** argv )
{
  
  // Default parameters. Don't touch this
  std::string     param_file                           = "param/cal_param.xml";
  std::string     cal_needed                           = "yes";
  double         cal_circular_pattern_radius          = 5.6;
  int            cal_circular_pattern_number_points   = 20;
  std::string     param_name_matrix                    = "matM";
  std::string     param_name_normal                    = "vecN";
  std::string     scan_output_file                     = "data/scan_output.xyz";
  double         scan_step_angle                      = 1;

  if(argc>1)
  {
  
    // Read parameters
    std::string line;
    std::string keyword;
    std::stringstream sline;
    
    std::fstream file(argv[1],std::ios::in);
    if(file)
    {
      while(getline(file,line))
      {
        if(line != "") // avoids empty lines
        {
          sline.clear();
          sline.str("");
          sline << line;
          sline >> keyword;
          // Parser
          if(keyword[0] == '#')
          {
            if      (keyword == "#param_file")
            {
              sline >> param_file;
            }
            else if (keyword == "#cal_needed")
            {
              sline >> cal_needed;
            }
            else if (keyword == "#cal_circular_pattern_radius")
            {
              sline >> cal_circular_pattern_radius;
            }
            else if (keyword == "#cal_circular_pattern_number_points")
            {
              sline >> cal_circular_pattern_number_points;
            }
            else if (keyword == "#param_name_matrix")
            {
              sline >> param_name_matrix;
            }
            else if (keyword == "#param_name_normal")
            {
              sline >> param_name_normal;
            }
            else if (keyword == "#scan_output_file")
            {
              sline >> scan_output_file;
            }
            else if (keyword == "#scan_step_angle")
            {
              sline >> scan_step_angle;
            }
            else
            {
            
            }
          }
        }
      }
    }
  }
  else
  {
    std::cerr << "Usage: ./scan scan_param.txt" << std::endl;
    exit(-1);
  }
  
  // Recap parameters
  std::cout << " 1) Parameters: " << std::endl;
  std::cout << "     param_file                           " << param_file << std::endl;
  std::cout << "     cal_needed                           " << cal_needed << std::endl;
  std::cout << "     cal_circular_pattern_radius          " << cal_circular_pattern_radius << std::endl;
  std::cout << "     cal_circular_pattern_number_points   " << cal_circular_pattern_number_points << std::endl;
  std::cout << "     param_name_matrix                    " << param_name_matrix << std::endl;
  std::cout << "     param_name_normal                    " << param_name_normal << std::endl;
  std::cout << "     scan_output_file                     " << scan_output_file << std::endl;
  std::cout << "     scan_step_angle                      " << scan_step_angle << std::endl;
  
  if(cal_needed=="yes")
  {
    Calibration cal;
    cal.circularPattern(cal_circular_pattern_number_points,cal_circular_pattern_radius);
    DataHandler dh;
//    std::vector<std::string> varNames;
//    std::vector<cv::Mat> vars;
//    
//    varNames.push_back("matM");
//    varNames.push_back("vecN");
//    
//    vars.push_back(cal.getMat());
//    vars.push_back(cal.getVec());
//    dc.saveInXML(param_file, varNames, vars);
    std::vector< std::vector<double> > matt = cal.getMatSTD();
    std::vector<double> vect = cal.getVecSTD();
    dh.saveMatrix3x3(param_file, std::string("matM"), matt);
    dh.saveVector3(param_file, std::string("vecN"), vect);
  }
  
  std::cout << " 2) Scan" << std::endl;
  Scan scan(param_file,scan_output_file,scan_step_angle);
  scan.read(param_name_matrix,param_name_normal);
  scan.launch();
  scan.save();
  
//  pcl::PointCloud<pcl::PointXYZ> cloud;
//  pcl::PointCloud<pcl::PointXYZ> cloudPP;
//  DataConverter dc;
  DataHandler dh;  
  // Convert scanned data to cloud
  std::vector< std::vector<double> > dat=scan.getData();
//  dc.convert(dat,cloud);
  
  
//  std::cout << " 4) Save data in " << post_proc_output_file << std::endl;
//  dc.save(post_proc_output_file, cloudPP);
  std::cout << " 3) Save data in " << scan_output_file << std::endl;
//  dc.save(scan_output_file, cloud);
  dh.saveAsXYZ(scan_output_file, dat);  
  return 0;
}



