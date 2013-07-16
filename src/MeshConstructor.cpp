#include "MeshConstructor.h"

MeshConstructor::MeshConstructor()
{

}

// Mesh a cloud without normals with the Greedy Projection Triangulation  method

void MeshConstructor::cloudToMeshGPT(  pcl::PointCloud<pcl::PointXYZ> & inputCloud, 
                                        pcl::PolygonMesh & outputMesh,
                                        double kSearch,
                                        double search_radius,
                                        double neigb_mu_crit,
                                        double mnn_crit,
                                        double max_surf_ang_crit,
                                        double min_ang_crit,
                                        double max_ang_crit)
{
//  cloudToMeshGPT(&inputCloud, &outputMesh, search_radius, neigb_mu_crit, mnn_crit, max_surf_ang_crit, min_ang_crit, max_ang_crit);
}
         
                        
void MeshConstructor::cloudToMeshGPT(  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
                                        pcl::PolygonMesh::Ptr outputMesh,
                                        double kSearch,
                                        double search_radius,
                                        double neigb_mu_crit,
                                        double mnn_crit,
                                        double max_surf_ang_crit,
                                        double min_ang_crit,
                                        double max_ang_crit)
{
  
  // Normal estimation
  std::cerr << " Estimate normal vectors" << std::endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud (cloud);
//  n.setInputCloud (cloud);
  tree->setInputCloud (inputCloud);
  n.setInputCloud (inputCloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
//  n.setRadiusSearch(search_radius);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*inputCloud, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);


  // Surface reconstruction
  std::cerr << " Reconstruct surface (greedy projection triangulation method)" << std::endl;
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//  pcl::PolygonMesh triangles;
  gp3.setSearchRadius (search_radius);
  gp3.setMu (neigb_mu_crit);
  gp3.setMaximumNearestNeighbors (mnn_crit);
  gp3.setMaximumSurfaceAngle(max_surf_ang_crit);
  gp3.setMinimumAngle(min_ang_crit);
  gp3.setMaximumAngle(max_ang_crit);
  gp3.setNormalConsistency(false);

  
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (*outputMesh);



}


void MeshConstructor::cloudToMeshMC(  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
                                      pcl::PolygonMesh::Ptr outputMesh,
                                      float isoLevel,
                                      int resx,
                                      int resy,
                                      int resz
                                      )
{
  
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
  copyPointCloud(*inputCloud, *inputCloudRGB);
//	// Normal estimation*
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> norm_est;
//	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//	tree->setInputCloud (inputCloudRGB);
//	norm_est.setInputCloud (inputCloudRGB);
//	norm_est.setSearchMethod (tree);
//	norm_est.setKSearch (30);
//	norm_est.compute (*ptn);
//	pcl::copyPointCloud (*inputCloudRGB, *ptn);
//	inputCloudRGB.reset();

//////////    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//////////    copyPointCloud(*inputCloud, *vertices);
//////////     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
//////////     normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
//////////     normal_estimation.setRadiusSearch (0.01);
//////////     normal_estimation.setInputCloud (inputCloudRGB);
//////////     normal_estimation.compute (*vertices);
////////////     feature_from_normals->setInputNormals(normals);


//////////  // Create the search method
//////////  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
//////////  tree2->setInputCloud (*vertices);
//////////  // Initialize objects
////////////  pcl::MarchingCubes<pcl::PointXYZRGBNormal> mc;
//////////  
//////////  // Set parameters
//////////  //	 mc.setLeafSize(leafSize);  
////////////  mc.setGridResolution(resx, resy, resz);
////////////  mc.setIsoLevel(isoLevel);   //ISO: must be between 0 and 1.0
////////////  mc.setSearchMethod(tree2);
////////////  mc.setInputCloud(ptn);
////////////  // Reconstruct
////////////  mc.reconstruct (*outputMesh);
//////////pcl::MarchingCubes<pcl::PointXYZRGBNormal>* mc = new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>;
//////////     mc->setSearchMethod(tree2);
//////////     mc->setInputCloud(vertices);
//////////     mc->setIsoLevel(isoLevel);
//////////     mc->setGridResolution(resx,resy,resz);
//////////     mc->reconstruct(*outputMesh);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::copyPointCloud(*inputCloudRGB, *vertices);
 
   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
   normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
   normal_estimation.setRadiusSearch (0.01);
   normal_estimation.setInputCloud (inputCloudRGB);
   normal_estimation.compute (*vertices);
   
   pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
   tree->setInputCloud (vertices);
 
   pcl::MarchingCubes<pcl::PointXYZRGBNormal>* mc = new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>;
   mc->setSearchMethod(tree);
   mc->setInputCloud(vertices);
   mc->reconstruct(mesh);

}




