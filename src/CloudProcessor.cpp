#include "CloudProcessor.h"

CloudProcessor::CloudProcessor()
{

}

void CloudProcessor::removeOutliers(PointList & points, const double removed_percentage, const int nb_neighbors)
{
  // Removes outliers using erase-remove idiom.
  // The Dereference_property_map property map can be omitted here as it is the default value.
  // removed_percentage = 5.0; // percentage of points to remove
  // nb_neighbors = 24; // considers 24 nearest neighbor points
  // std::cerr << "2) Remove outliers (" << removed_percentage << "%)" << std::endl;
  points.erase(CGAL::remove_outliers(points.begin(), points.end(),
                                     CGAL::Dereference_property_map<Point>(),
                                     nb_neighbors, removed_percentage), 
               points.end());

  // Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
  std::vector<Point>(points).swap(points);
}




void CloudProcessor::simplifyCloud(PointList & points, double cell_size)
{
  //  std::cerr << "3) Simplify cloud" << std::endl;
  //  cell_size = 0.001;
  points.erase(CGAL::grid_simplify_point_set(points.begin(), points.end(), cell_size),
               points.end());

  // Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
  std::vector<Point>(points).swap(points);
}




void CloudProcessor::smoothCloud(PointList & points, int iter, const int nb_neighbors)
{
  // nb_neighbors = 24; // considers 24 nearest neighbor points
  for(int k=0;k<iter;++k)
    CGAL::jet_smooth_point_set(points.begin(), points.end(), nb_neighbors);
}



// pointsWN must be empty
void CloudProcessor::estimateNormals(PointList & points, PointWNList & pointsWN, const int nb_neighbors)
{
  // Fill the point field of pointsWN
  for(unsigned long int i=0;i<points.size();++i)
  {
    PointWN p;
    p.first=points[i];
    Vector v;
    p.second=v;
    pointsWN.push_back(p);
  }


  // trying to adapt types
  
  CGAL::pca_estimate_normals(pointsWN.begin(), pointsWN.end(),
                             CGAL::First_of_pair_property_map<std::pair<Point, Vector> >(),
                             CGAL::Second_of_pair_property_map<std::pair<Point, Vector> >(),
                             nb_neighbors);
}


void CloudProcessor::orientNormals(PointWNList & pointsWN, const int nb_neighbors)
{
  // Orients normals.
  // Note: mst_orient_normals() requires an iterator over points
  // as well as property maps to access each point's position and normal.
  PointWNList::iterator unoriented_points_begin = CGAL::mst_orient_normals(pointsWN.begin(), pointsWN.end(),
                                                                           CGAL::First_of_pair_property_map<std::pair<Point, Vector> >(),
                                                                           CGAL::Second_of_pair_property_map<std::pair<Point, Vector> >(),
                                                                           nb_neighbors);
  
//    // Optional: delete points with an unoriented normal
//    // if you plan to call a reconstruction algorithm that expects oriented normals.
  pointsWN.erase(unoriented_points_begin, pointsWN.end());

//    // Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
  PointWNList(pointsWN).swap(pointsWN);

}

FT CloudProcessor::computeAverageSpacing(PointList & points)
{
  return (CGAL::compute_average_spacing(points.begin(), points.end(), 6 /* knn = 1 ring */));
}



