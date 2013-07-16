#ifndef MESHCONSTRUCTOR_H
#define MESHCONSTRUCTOR_H

#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_utils.h>


class MeshConstructor
{
protected:
  


public:
  MeshConstructor();
  void cloudToMeshGPT( pcl::PointCloud<pcl::PointXYZ> & inputCloud, 
                        pcl::PolygonMesh & outputMesh,
                        double kSearch,
                        double search_radius,
                        double neigb_mu_crit,
                        double mnn_crit,
                        double max_surf_ang_crit,
                        double min_ang_crit,
                        double max_ang_crit);
                        
  void cloudToMeshGPT( pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
                        pcl::PolygonMesh::Ptr outputMesh,
                        double kSearch,
                        double search_radius,
                        double neigb_mu_crit,
                        double mnn_crit,
                        double max_surf_ang_crit,
                        double min_ang_crit,
                        double max_ang_crit);
  

  void cloudToMeshMC(  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
                        pcl::PolygonMesh::Ptr outputMesh,
                        float isoLevel,
                        int resx,
                        int resy,
                        int resz
                        );


};


#endif
