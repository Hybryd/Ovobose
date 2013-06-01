#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <stdlib.h>

#include "../DataConverter.h"
#include "../PostProcessor.h"


int main(int argc, char ** argv)
{
  std::string     post_proc_input_file                 = "data/data.xyz";
  std::string     post_proc_output_file                = "data/data_PP.xyz";
  pcl::PointXYZ   post_proc_cylinder_center            = pcl::PointXYZ(0,0,0);
  double         post_proc_cylinder_radius            = 7;
  double         post_proc_cylinder_height            = 15;

  if(argc < 2)
  {
    std::cout << "Usage: ./postProcess postProcess_param.txt" << std::endl;
    return (-1);
  }
  
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
            if      (keyword == "#post_proc_input_file")
            {
              sline >> post_proc_input_file;
            }
            else if (keyword == "#post_proc_output_file")
            {
              sline >> post_proc_output_file;
            }
            else if (keyword == "#post_proc_cylinder_center")
            {
              double x,y,z;
              sline >> x >> y >> z;
              post_proc_cylinder_center = pcl::PointXYZ(x,y,z);
            }
            else if (keyword == "#post_proc_cylinder_radius")
            {
              sline >> post_proc_cylinder_radius;
            }
            else if (keyword == "#post_proc_output_file")
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
  
  pcl::PointCloud<pcl::PointXYZ> cloud;     // raw data
  pcl::PointCloud<pcl::PointXYZ> cloudPP;  // post processed data
  DataConverter dc;                         // converter
  
  std::cerr << " 1) Read " << post_proc_input_file << std::endl;
  dc.read(post_proc_input_file, cloud);


  std::cerr << " 2) Post processing data " << std::endl;
  PostProcessor p;//(cloud, post_proc_output_file);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPP(new pcl::PointCloud<pcl::PointXYZ>);// = p.getDataPP();
  p.keepInCylinder(cloud, cloudPP, post_proc_cylinder_center, post_proc_cylinder_radius, post_proc_cylinder_height);
  
  std::cerr << " 3) Save post processed data in " << post_proc_output_file << std::endl;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPP = p.getDataPP();
  dc.save(post_proc_output_file, cloudPP);

  

  return (0);
}
