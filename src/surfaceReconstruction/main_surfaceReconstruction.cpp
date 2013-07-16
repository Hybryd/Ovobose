
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../DataConverter.h"
#include "../MeshConstructor.h"
#include "../PostProcessor.h"
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_utils.h>



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
  bool qu_reconstruct             = true; // do you want to reconstruct the surface
  double vog_leafsize             = .05;
  double sor_outl_tolerance       = 0.5;
  double sor_meanK                = 50;
  double mls_search_radius        = .1;
  bool mls_polygonal_fit          = true;
  int mls_nbIter                  = 1;
  double gp_kSearch               = 20;
  double gp_search_radius         = .25;
  double gp_neigb_mu_crit         = 2;
  double gp_mnn_crit              = 500;
  double gp_max_surf_ang_crit     = M_PI/4;
  double gp_min_ang_crit          = M_PI/18;
  double gp_max_ang_crit          = 2*M_PI/3;
  double ne_radius                = 0.1;
  unsigned long int ms_nbIter    = 60000;
  double ms_convergence           = 0.0001;
  double ms_relaxFactor           = 0.0001;
  bool ms_edgeSmoothing           = true;
  double ms_angle                 = 10;
  bool ms_boundarySmoothing       = true;
  float mc_leafSize               = 0.1;
  double mc_isoLevel              = 0.001;
  int mc_resx                      = 50;
  int mc_resy                      = 50;
  int mc_resz                      = 50;



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
            else if (keyword == "#qu_reconstruct")
            {
              std::string s;
              sline >> s;
              qu_reconstruct = (s=="true");
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
            else if (keyword == "#mls_polygonal_fit")
            {
              std::string s;
              sline >> s;
              mls_polygonal_fit = (s=="true");
            }
            else if (keyword == "#mls_nbIter")
            {
              sline >> mls_nbIter;
            }
            else if (keyword == "#gp_kSearch")
            {
              sline >> gp_kSearch;
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
            else if (keyword == "#ms_nbIter")
            {
              sline >> ms_nbIter;
            }
            else if (keyword == "#ms_convergence")
            {
              sline >> ms_convergence;
            }
            else if (keyword == "#ms_relaxFactor")
            {
              sline >> ms_relaxFactor;
            }
            else if (keyword == "#ms_edgeSmoothing")
            {
              std::string s;
              sline >> s;
              ms_edgeSmoothing = (s=="true");
            }
            else if (keyword == "#ms_angle")
            {
              sline >> ms_angle;
              ms_angle = M_PI/ms_angle;
            }
            else if (keyword == "#ms_boundarySmoothing")
            {
              std::string s;
              sline >> s;
              ms_boundarySmoothing = (s=="true");
            }
            else if (keyword == "#mc_leafSize")
            {
              sline >> mc_leafSize;
            }
            else if (keyword == "#mc_isoLevel")
            {
              sline >> mc_isoLevel;
            }
            else if (keyword == "#mc_resolution")
            {
              sline >> mc_resx >> mc_resy >> mc_resz;
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
  
  if(output_name.find(".pcd") == output_name.size()-4 || output_name.find(".vtk") == output_name.size()-4 || output_name.find(".ply") == output_name.size()-4 || output_name.find(".obj") == output_name.size()-4 || output_name.find(".stl") == output_name.size()-4)
  {
    // ok
  }
  else //if(output_name.find(".vtk") != output_name.size()-4 )
  {
    output_name += ".vtk";
    std::cerr << "WARNING: data will be saved in " << output_name << "." << std::endl;
  }
  
  std::cout << "    input_file           " << input_name << std::endl;
  std::cout << "    output_file          " << output_name << std::endl;
  std::cout << "    vog_leafsize         " << vog_leafsize << std::endl;
  std::cout << "    sor_outl_tolerance   " << sor_outl_tolerance << std::endl;
  std::cout << "    sor_meanK            " << sor_meanK << std::endl;
  std::cout << "    mls_search_radius    " << mls_search_radius << std::endl;
  std::cout << "    mls_search_radius    " << mls_polygonal_fit << std::endl;
  std::cout << "    mls_nbIter           " << mls_nbIter << std::endl;
  std::cout << "    gp_kSearch           " << gp_kSearch << std::endl;
  std::cout << "    gp_search_radius     " << gp_search_radius << std::endl;
  std::cout << "    gp_neigb_mu_crit     " << gp_neigb_mu_crit << std::endl;
  std::cout << "    gp_mnn_crit          " << gp_mnn_crit << std::endl;
  std::cout << "    gp_max_surf_ang_crit " << gp_max_surf_ang_crit << std::endl;
  std::cout << "    gp_min_ang_crit      " << gp_min_ang_crit << std::endl;
  std::cout << "    gp_max_ang_crit      " << gp_max_ang_crit << std::endl;
  std::cout << "    ne_radius            " << ne_radius << std::endl;
  std::cout << "    ms_nbIter            " << ms_nbIter << std::endl;
  std::cout << "    ms_convergence       " << ms_convergence << std::endl;
  std::cout << "    ms_relaxFactor       " << ms_relaxFactor << std::endl;
  std::cout << "    ms_edgeSmoothing     " << ms_edgeSmoothing << std::endl;
  std::cout << "    ms_angle             " << ms_angle << std::endl;
  std::cout << "    ms_boundarySmoothing " << ms_boundarySmoothing << std::endl;
  std::cout << "    mc_leafSize          " << mc_leafSize << std::endl;
  std::cout << "    mc_isoLevel          " << mc_isoLevel << std::endl;
  std::cout << "    mc_resolution        " << mc_resx << " " << mc_resy << " " << mc_resz  << std::endl;
  

  // Load input file into a PointCloud<T> with an appropriate type
  std::cerr << " 2) Read data from file " << input_name << std::endl;
  DataConverter dc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  dc.read(input_name, cloud);

  // Preprocessing
  std::cerr << " 3) Preprocess data" << std::endl;
  PostProcessor pp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered0 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Removing noise and points in overly dense area
  pp.smoothCloudSOR(cloud, cloud_filtered0, sor_meanK, sor_outl_tolerance);
  
  // Smooth cloud
  pp.smoothCloud(cloud_filtered0, cloud_filtered, mls_polygonal_fit, mls_search_radius, mls_nbIter);
  
  if(qu_reconstruct)
    {
    
      // Mesh construction
      std::cerr << " 4) Surface reconstruction" << std::endl;
      MeshConstructor meshconst;
      pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
      meshconst.cloudToMeshGPT(cloud_filtered, triangles, gp_kSearch, gp_search_radius, gp_neigb_mu_crit, gp_mnn_crit, gp_max_surf_ang_crit, gp_min_ang_crit, gp_max_ang_crit);
    //  meshconst.cloudToMeshMC(cloud, triangles, mc_isoLevel, mc_resx, mc_resy, mc_resz );

      // Mesh postprocessing
      std::cerr << " 6) Postprocess mesh (Laplacian VTKSmoothing)" << std::endl;
    //  PostProcessor pp;
      pcl::PolygonMesh::Ptr inputMesh(triangles);
      pcl::PolygonMesh outputMesh;
      pp.smoothMesh(inputMesh, outputMesh, ms_nbIter, ms_convergence, ms_relaxFactor, ms_edgeSmoothing, ms_angle, ms_boundarySmoothing);
      

      // Save data
      
      std::string nameraw=output_name.substr(0,output_name.find("."))+"_raw"+output_name.substr(output_name.find("."),output_name.size()-1);
      std::string namesmo=output_name.substr(0,output_name.find("."))+"_smo"+output_name.substr(output_name.find("."),output_name.size()-1);
      std::cerr << " 7) Store data in " << nameraw << " and " << namesmo << std::endl;
      dc.savePolygon(nameraw, *triangles);
      dc.savePolygon(namesmo, outputMesh);
      
    //  // Visualize
    //  cout << " 8) Visualize " << endl;
    //  
    //  pcl::visualization::PCLVisualizer *p;
    //  p = new pcl::visualization::PCLVisualizer (argc, argv, "Surface reconstructed from scanned cloud point");
    //  int vPort1 = 1;
    //  int vPort2 = 2;
    ////  p->removePolygonMesh("meshIn", vPort1);
    ////  p->removePolygonMesh("meshSmoothed", vPort2);
    //  p->createViewPort (0.0, 0, 0.5, 1.0, vPort1);
    ////  p->createViewPort (0.5, 0, 1.0, 1.0, vPort2);
    //  p->addPolygonMesh(triangles, "meshIn", vPort1);
    ////  p->addPolygonMesh(*outputMesh, "meshSmoothed", vPort2);
    //  p->resetCameraViewpoint();
    //  p->spin(); 
  }
  else
  {
    std::cerr << " 4) Store data in " << output_name << std::endl;
    dc.save(output_name, cloud_filtered);
  }   
  std::cerr << "Done!" << std::endl;
  
  return (0);
}
