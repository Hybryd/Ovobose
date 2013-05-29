#include "Calibration.h"
#include "PostProcessor.h"
#include "Scan.h"
#include <stdio.h>
#include <stdlib.h>
//#include <iostream>
//#include <sstream>

// Usage ./main [scan.param]

int main(int argc, char ** argv )
{
  
  // Default parameters. Don't touch this
  std::string     param_file                           = "cal_param.xml";
  std::string     cal_needed                           = "yes";
  double         cal_circular_pattern_radius          = 5.6;
  int            cal_circular_pattern_number_points   = 20;
  std::string     param_name_matrix                    = "matM";
  std::string     param_name_normal                    = "vecN";
  std::string     scan_output_file                     = "data/scan_output.xyz";
  double         scan_step_angle                      = 1;
  std::string     post_proc_output_file                = "data/scan_output_PP.xyz";
  double         post_proc_cylinder_radius            = 7;
  double         post_proc_cylinder_height            = 15;

  
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
            else if (keyword == "#post_proc_output_file")
            {
              sline >> post_proc_output_file;
            }
            else if (keyword == "#post_proc_cylinder_radius")
            {
              sline >> post_proc_cylinder_radius;
            }
            else if (keyword == "#post_proc_cylinder_height")
            {
              sline >> post_proc_cylinder_height;
            }
            else
            {
            
            }
          }
        }
      }
    }
  }
//  else
//  {
//    std::cerr << "Scanning..." << std::endl;
//    exit(-1);
//  }
  
  // Recap parameters
  std::cout << "Parameters: " << std::endl;
  std::cout << " param_file                           " << param_file << std::endl;
  std::cout << " cal_needed                           " << cal_needed << std::endl;
  std::cout << " cal_circular_pattern_radius          " << cal_circular_pattern_radius << std::endl;
  std::cout << " cal_circular_pattern_number_points   " << cal_circular_pattern_number_points << std::endl;
  std::cout << " param_name_matrix                    " << param_name_matrix << std::endl;
  std::cout << " param_name_normal                    " << param_name_normal << std::endl;
  std::cout << " scan_output_file                     " << scan_output_file << std::endl;
  std::cout << " scan_step_angle                      " << scan_step_angle << std::endl;
  std::cout << " post_proc_output_file                " << post_proc_output_file << std::endl;
  std::cout << " post_proc_cylinder_radius            " << post_proc_cylinder_radius << std::endl;
  std::cout << " post_proc_cylinder_height            " << post_proc_cylinder_height << std::endl;
  
  if(cal_needed=="yes")
  {
    Calibration cal(param_file);
    cal.circularPattern(cal_circular_pattern_number_points,cal_circular_pattern_radius);
    cal.save(param_name_matrix,param_name_normal);
  }
  
  std::cout << "Scanning..." << std::endl;
  Scan scan(param_file,scan_output_file,scan_step_angle);
  scan.read(param_name_matrix,param_name_normal);
  scan.launch();
  scan.save();
  
  std::cout << "Post processing..." << std::endl;
  PostProcessor p(scan.getData(), post_proc_output_file);
  p.keepInCylinder(cv::Point3d(0,0,0), post_proc_cylinder_radius, post_proc_cylinder_height);
  p.save();
  
  
  
  return 0;
}
