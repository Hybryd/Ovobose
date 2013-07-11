
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../DataConverter.h"
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>



// Usage: ./surfaceReconstruction surf.param

int main (int argc, char** argv)
{
  if(argc < 2)
  {
    std::cerr << "Usage: ./surfaceReconstruction surf.param" << std::endl;
    exit(-1);
  }

  std::string input_name           = "data/test.pcd"; // pcd, ply or xyz file
  std::string output_name          = "data/test.vtk"; // vtk file
  double vog_leafsize             = .05;
  double sor_outl_tolerance       = 0.5;
  double sor_meanK                = 50;
  double mls_search_radius        = .1;
  double gp_search_radius         = .25;
  double gp_neigb_mu_crit         = 2;
  double gp_mnn_crit              = 500;
  double gp_max_surf_ang_crit     = M_PI/4;
  double gp_min_ang_crit          = M_PI/18;
  double gp_max_ang_crit          = 2*M_PI/3;
  double ne_radius                = 0.1;



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
            if      (keyword == "#input_name")
            {
              sline >> input_name;
            }
            else if (keyword == "#output_name")
            {
              sline >> output_name;
            }
            else if (keyword == "#vog_leafsize")
            {
              sline >> vog_leafsize;
            }
            else if (keyword == "#sor_outl_tolerance")
            {
              sline >> sor_outl_tolerance;
            }
            else if (keyword == "#sor_meanK")
            {
              sline >> sor_meanK;
            }
            else if (keyword == "#mls_search_radius")
            {
              sline >> mls_search_radius;
            }
            else if (keyword == "#gp_search_radius")
            {
              sline >> gp_search_radius;
            }
            else if (keyword == "#gp_neigb_mu_crit")
            {
              sline >> gp_neigb_mu_crit;
            }
            else if (keyword == "#gp_mnn_crit")
            {
              sline >> gp_mnn_crit;
            }
            else if (keyword == "#gp_max_surf_ang_crit")
            {
              sline >> gp_max_surf_ang_crit;
              gp_max_surf_ang_crit = M_PI/gp_max_surf_ang_crit;
            }
            else if (keyword == "#gp_min_ang_crit")
            {
              sline >> gp_min_ang_crit;
              gp_min_ang_crit = M_PI/gp_min_ang_crit;
            }
            else if (keyword == "#gp_max_ang_crit")
            {
              sline >> gp_max_ang_crit;
              gp_max_ang_crit = M_PI/gp_max_ang_crit;
            }
            else if (keyword == "#ne_radius")
            {
              sline >> ne_radius;
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
      std::cerr << " 1) WARNING: unable to open " << output_name << ". Default parameters are loaded" << std::endl;
    }
  }
  else
  {
    std::cerr << " 1) WARNING: Default parameters are loaded" << std::endl;
  }
  
    // Check if output file is .vtk
  
  if(output_name.find(".vtk") == output_name.size()-4 || output_name.find(".ply") == output_name.size()-4 || output_name.find(".stl") == output_name.size()-4)
  {
    // ok
  }
  else //if(output_name.find(".vtk") != output_name.size()-4 )
  {
    output_name += ".stl";
    std::cerr << "WARNING: output file must be STL. Data will be saved in " << output_name << "." << std::endl;
  }
  
  std::cout << "    input_file           " << input_name << std::endl;
  std::cout << "    output_file          " << output_name << std::endl;
  std::cout << "    vog_leafsize         " << vog_leafsize << std::endl;
  std::cout << "    sor_outl_tolerance   " << sor_outl_tolerance << std::endl;
  std::cout << "    sor_meanK            " << sor_meanK << std::endl;
  std::cout << "    mls_search_radius    " << mls_search_radius << std::endl;
  std::cout << "    gp_search_radius     " << gp_search_radius << std::endl;
  std::cout << "    gp_neigb_mu_crit     " << gp_neigb_mu_crit << std::endl;
  std::cout << "    gp_mnn_crit          " << gp_mnn_crit << std::endl;
  std::cout << "    gp_max_surf_ang_crit " << gp_max_surf_ang_crit << std::endl;
  std::cout << "    gp_min_ang_crit      " << gp_min_ang_crit << std::endl;
  std::cout << "    gp_max_ang_crit      " << gp_max_ang_crit << std::endl;
  std::cout << "    ne_radius            " << ne_radius << std::endl;
  
  

  // Load input file into a PointCloud<T> with an appropriate type
  std::cout << " 2) Read data from file " << input_name << std::endl;
  DataConverter dc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  dc.read(input_name, cloud);
//  sensor_msgs::PointCloud2 cloud_blob;
//  pcl::io::loadPCDFile (input_name, cloud_blob);
//  pcl::fromROSMsg (cloud_blob, *cloud);
  
  

  // Removing noise and points in overly dense area
  std::cout << " 3) Preprocess data" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vog;
  vog.setInputCloud (cloud);
  vog.setLeafSize (vog_leafsize, vog_leafsize, vog_leafsize);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (sor_meanK);
  sor.setStddevMulThresh (sor_outl_tolerance);

  sor.filter (*cloud_filtered);
  vog.filter (*cloud_filtered);


  

  // Normal estimation
  std::cout << " 4) Estimate normal vectors" << std::endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud (cloud);
//  n.setInputCloud (cloud);
  tree->setInputCloud (cloud_filtered);
  n.setInputCloud (cloud_filtered);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;
  gp3.setSearchRadius (gp_search_radius);
  gp3.setMu (gp_neigb_mu_crit);
  gp3.setMaximumNearestNeighbors (gp_mnn_crit);
  gp3.setMaximumSurfaceAngle(gp_max_surf_ang_crit);
  gp3.setMinimumAngle(gp_min_ang_crit);
  gp3.setMaximumAngle(gp_max_ang_crit);
  gp3.setNormalConsistency(false);

  // Surface reconstruction
  cout << " 5) Reconstruct surface (greedy projection triangulation method)" << endl;
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

//  // Additional vertex information
//  std::vector<int> parts = gp3.getPartIDs();
//  std::vector<int> states = gp3.getPointStates();

  // Save data
  cout << " 6) Store data in " << output_name << endl;
//  pcl::io::savePolygonFileVTK(output_name, triangles);
  dc.savePolygon(output_name, triangles);
  
  return (0);
}
