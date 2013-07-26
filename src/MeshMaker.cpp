#include "MeshMaker.h"

MeshMaker::MeshMaker()
{

}


//FT sm_angle = 20.0; // Min triangle angle in degrees.
//FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
//FT sm_distance = 0.375; // Surface Approximation error w.r.t. point set average spacing.
void MeshMaker::poissonReconstruction(PointList & points, Polyhedron & poly, const int nbNeig, FT sm_angle, FT sm_radius, FT sm_distance)
{
  CloudProcessor cp;
  // Compute average spacing
  FT average_spacing = cp.computeAverageSpacing(points);

  // Make PointWNList
  PointWNList pointsWN;
  
  std::cerr << "  Estimate normals" << std::endl;
  cp.estimateNormals(points, pointsWN, nbNeig);
  cp.orientNormals(pointsWN,nbNeig);

  
  Poisson_reconstruction_function function( pointsWN.begin(), 
                                            pointsWN.end(),
                                            //CGAL::make_normal_of_point_with_normal_pmap(pointsWN.begin())
                                             CGAL::First_of_pair_property_map<PointWN>(),
                                             CGAL::Second_of_pair_property_map<PointWN>()
                                          );

  // Computes the Poisson indicator function f()
  // at each vertex of the triangulation.
  if ( ! function.compute_implicit_function() )
  {
    std::cerr << "ERROR [MeshMaker::PoissonReconstruction]: implicit function not computed.";
    exit(0);
//    return EXIT_FAILURE;
  }

  // Gets one point inside the implicit surface
  // and computes implicit function bounding sphere radius.
  Point inner_point = function.get_inner_point();
  Sphere bsphere = function.bounding_sphere();
  FT radius = std::sqrt(bsphere.squared_radius());

  // Defines the implicit surface: requires defining a
  // conservative bounding sphere centered at inner point.
  FT sm_sphere_radius = 5.0 * radius;
  FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
  Surface_3 surface(function,
                    Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                    sm_dichotomy_error/sm_sphere_radius);

  // Defines surface mesh generation criteria
  CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                      sm_radius*average_spacing,  // Max triangle size
                                                      sm_distance*average_spacing); // Approximation error

  std::cerr << "  Reconstruct surface" << std::endl;
  // Generates surface mesh with manifold option
  STr tr; // 3D Delaunay triangulation for surface mesh generation
  C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
  CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                          surface,                              // implicit surface
                          criteria,                             // meshing criteria
                          CGAL::Manifold_with_boundary_tag());  // require manifold mesh

  if(tr.number_of_vertices() == 0)
  {
    std::cerr << "ERROR [MeshMaker::PoissonReconstruction]: no vertices generated.";
    exit(0);
//    return EXIT_FAILURE;
  }
  else
  {
    std::cerr << "  Conversion" << std::endl;
    CGAL::output_surface_facets_to_polyhedron(c2t3, poly);
  }
}

