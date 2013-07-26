#include "DataHandler.h"

DataHandler::DataHandler()
{
}


void DataHandler::readXYZ(std::string fileName, PointList & cloud)
{
//  if (fileName.find(".xyz") == fileName.size()-4)
  {
    std::ifstream input(fileName.c_str());
    if(!input.is_open())
    {
      std::cerr << "ERROR [ DataHandler::readXYZ ]: unable to open "<< fileName << std::endl;
      exit(-1);
    }
    std::string line;
    double x,y,z;
    while(getline(input,line))
    {
      input >> x >> y >> z;
      cloud.push_back(Point(x,y,z));
    }
    input.close();
  }
//  else
//  {
//    std::cerr << "ERROR: "<< fileName << " must end with .pcd, .ply or .xyz" << std::endl;
//  }
}




void DataHandler::saveAsXYZ(std::string fileName, std::vector< std::vector<double> > & vecMat)
{
  // Saves point set.
  // Note: write_xyz_points_and_normals() requires an output iterator
  // over points as well as property maps to access each
  // point position and normal.
  std::ofstream out(fileName.c_str());
  if(out)
  {
    for(unsigned long int i=0; i< vecMat.size(); ++i)
    {
      for(int j=0; j< vecMat[i].size(); ++j)
      {
        out << vecMat[i][j] << " ";
      }
      out << std::endl;
    }
  }
  else
    std::cerr << "ERROR [ DataHandler::saveAsXYZ ]: unable to write in " << fileName << std::endl;
}

void DataHandler::saveAsXYZ(std::string fileName, PointList & cloud)
{
  // Saves point set.
  // Note: write_xyz_points_and_normals() requires an output iterator
  // over points as well as property maps to access each
  // point position and normal.
  std::ofstream out(fileName.c_str());
  if(out)
  {
    for(unsigned long int i=0; i< cloud.size(); ++i)
    {
      out << cloud[i][0] << " " << cloud[i][1] << " " << cloud[i][2] << std::endl;
    }
  }
  else
    std::cerr << "ERROR [ DataHandler::saveAsXYZ ]: unable to write in " << fileName << std::endl;
}


void DataHandler::saveAsXYZ(std::string fileName, PointWNList & cloudWN)
{
  // Saves point set.
  // Note: write_xyz_points_and_normals() requires an output iterator
  // over points as well as property maps to access each
  // point position and normal.
  std::ofstream out(fileName.c_str());
  if(out)
  {
    for(unsigned long int i=0; i< cloudWN.size(); ++i)
    {
      out << cloudWN[i].first[0] << " " << cloudWN[i].first[1] << " " << cloudWN[i].first[2] << " " << cloudWN[i].second[0] << " " << cloudWN[i].second[1] << " " << cloudWN[i].second[2] << std::endl;
    }
  }
  else
    std::cerr << "ERROR [ DataHandler::saveAsXYZ ]: unable to write in " << fileName << std::endl;
}

void DataHandler::saveAsOFF(std::string fileName, CGAL::Polyhedron_3<Kernel> & poly)
{
  std::ofstream out(fileName.c_str());
//    Polyhedron output_mesh;
//    CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);
  if(out)
    out << poly;
  else
    std::cerr << "ERROR [ DataHandler::saveAsOFF ]: unable to write in " << fileName << std::endl;
}


void DataHandler::saveAsOFF(std::string fileName, CGAL::Surface_mesh_complex_2_in_triangulation_3<CGAL::Surface_mesh_default_triangulation_3> & cplx)
{
  std::ofstream out(fileName.c_str());
  CGAL::Polyhedron_3<Kernel> output_mesh;
  CGAL::output_surface_facets_to_polyhedron(cplx, output_mesh);
  saveAsOFF(fileName, output_mesh);
}
