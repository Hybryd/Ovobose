
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include "../DataHandler.h"
#include "../CloudProcessor.h"
#include "../MeshMaker.h"
#include "../types.h"


// Usage: ./surfaceReconstruction surf.param

int main (int argc, char** argv)
{
  if(argc < 2)
  {
    std::cerr << "Usage: ./bin/surfaceReconstruction ./param/surf.param" << std::endl;
    exit(-1);
  }

  std::string input_cloud_file_name                     = "data/test.xyz";
  std::string output_cloud_with_normals_file_name       = "data/test_with_normals.xyzn";
  std::string output_mesh_file_name                     = "data/test_mesh.off";
  double      cp_cell_size                              = 0.001;  
  int         cp_nb_neighbors                           = 24;
  FT          mm_min_angle                              = 20;
  FT          mm_radius                                 = 30;
  FT          mm_distance                               = 0.375;
  
  // Read parameters
  std::string line;
  std::string keyword;
  std::stringstream sline;
  
  if(argc > 1)
  {
    std::fstream file(argv[1],std::ios::in);
    if(file)
    {
      std::cout << " 1) Read parameters from " << argv[1] << "." << std::endl;
      while(getline(file,line))
      {
        if(line != "")
        {
          sline.clear();
          sline.str("");
          sline << line;
          sline >> keyword;
          
          if(keyword[0] == '#')
          {
            if      (keyword == "#input_cloud_file_name")
            {
              sline >> input_cloud_file_name;
            }
            else if (keyword == "#output_cloud_with_normals_file_name")
            {
              sline >> output_cloud_with_normals_file_name;
            }
            else if (keyword == "#output_mesh_file_name")
            {
              sline >> output_mesh_file_name;
            }
            else if (keyword == "#cp_cell_size")
            {
              sline >> cp_cell_size;
            }
            else if (keyword == "#cp_nb_neighbors")
            {
              sline >> cp_nb_neighbors;
            }
            else if (keyword == "#mm_min_angle")
            {
              sline >> mm_min_angle;
            }
            else if (keyword == "#mm_radius")
            {
              sline >> mm_radius;
            }
            else if (keyword == "#mm_distance")
            {
              sline >> mm_distance;
            }
            else
            {
            
            }
          }
        }
      }
    }
    else
    {
      std::cerr << " 1) WARNING: unable to open " << std::string(argv[1]) << ". Default parameters are loaded" << std::endl;
    }
  }
  else
  {
    std::cerr << " 1) WARNING: Default parameters are loaded" << std::endl;
  }
  
  
  std::cout << " File names                             " << std::endl;
  std::cout << "    input_cloud_file_name               " << input_cloud_file_name << std::endl;
  std::cout << "    output_cloud_with_normals_file_name " << output_cloud_with_normals_file_name << std::endl;
  std::cout << "    output_mesh_file_name               " << output_mesh_file_name << std::endl;
  std::cout << " Cloud processing                       " << std::endl;
  std::cout << "    cp_cell_size                        " << cp_cell_size << std::endl;
  std::cout << "    cp_nb_neighbors                     " << cp_nb_neighbors << std::endl;
  std::cout << " Mesh maker                             " << std::endl;
  std::cout << "    mm_min_angle                        " << mm_min_angle << std::endl;
  std::cout << "    mm_radius                           " << mm_radius << std::endl;
  std::cout << "    mm_distance                         " << mm_distance << std::endl;

  
  DataHandler dh;
  CloudProcessor cp;

  PointList cloud;
  PointWNList cloudWN;
  dh.readXYZ(input_cloud_file_name,cloud);

  std::cerr << "1) Simplify cloud " << std::endl;
  cp.simplifyCloud(cloud, cp_cell_size); // 0.001

  std::cerr << "2) Estimate normals " << std::endl;
  cp.estimateNormals(cloud, cloudWN, cp_nb_neighbors); // 24
  
  dh.saveAsXYZ(output_cloud_with_normals_file_name,cloudWN);
  
  std::cerr << "3) Surface reconstruction" << std::endl;
  Polyhedron poly;
  MeshMaker mm;
  mm.poissonReconstruction(cloud, poly, cp_nb_neighbors, mm_min_angle, mm_radius, mm_distance);
  
  dh.saveAsOFF(output_mesh_file_name,poly);
  
  std::cerr << "Done." << std::endl;
  std::cerr << "Files generated: " << output_cloud_with_normals_file_name << std::endl;
  std::cerr << "                 " << output_mesh_file_name << std::endl;
  
  return (0);
}
