#include "DataConverter.h"

DataConverter::DataConverter()
{
}

//DataConverter::DataConverter()
//{
//  empty=true;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr datatmp (new pcl::PointCloud<pcl::PointXYZ>);
//  data = datatmp;
//  empty=true;
//}

//DataConverter::DataConverter(std::string fileName)
//{
//  empty=true;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr datatmp (new pcl::PointCloud<pcl::PointXYZ>);
//  data = datatmp;
//  read(fileName);
//  empty=(data->size()==0);
//}

//DataConverter::DataConverter(pcl::PointCloud<pcl::PointXYZ>::Ptr pData)
//{
//  empty=true;
//  data=pData;
//  empty=(data->size()==0);
//}

//DataConverter::DataConverter(std::vector<cv::Mat> & vecMat)
//{
//  convert(vecMat);
//}


void DataConverter::convert(std::vector<cv::Mat> & vecMat, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
//  empty=true;
  for(unsigned int i=0; i<vecMat.size(); ++i)
  {
    cloud.push_back(pcl::PointXYZ(vecMat[i].at<double>(0,0),vecMat[i].at<double>(0,1),vecMat[i].at<double>(0,2)));
  }
//  empty=(data->size()==0);
}

void DataConverter::convert(std::vector<cv::Mat> & vecMat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // allocate memory if cloud is NULL
  if(cloud == NULL)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud2;
  }
  
  convert(vecMat, *cloud);
}


void DataConverter::convert(std::vector< std::vector<double> > & vecMat, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
//  empty=true;
  for(unsigned int i=0; i<vecMat.size(); ++i)
  {
//    for(unsigned int j=0; j<vecMat[i].size(); ++j)
//    {
      cloud.push_back(pcl::PointXYZ(vecMat[i][0],vecMat[i][1],vecMat[i][2]));
//    }
  }
//  empty=(data->size()==0);
}

void DataConverter::convert(std::vector< std::vector<double> > & vecMat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // allocate memory if cloud is NULL
  if(cloud == NULL)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud2;
  }
  convert(vecMat,*cloud);
}



void DataConverter::read(std::string fileName, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if      (fileName.find(".pcd") == fileName.size()-4)
  {
    pcl::io::loadPCDFile (fileName.c_str(), cloud);
  }
  else if (fileName.find(".ply") == fileName.size()-4)
  {
    pcl::io::loadPLYFile (fileName.c_str(), cloud);
  }
  else if (fileName.find(".xyz") == fileName.size()-4)
  {
    std::ifstream input(fileName.c_str());
    if(!input.is_open())
    {
      std::cerr << "ERROR: unable to open "<< fileName << std::endl;
      exit(-1);
    }
    std::string line;
    double x,y,z;
    while(getline(input,line))
    {
      input >> x >> y >> z;
      cloud.push_back(pcl::PointXYZ(x,y,z));
    }
    input.close();
  }
  else
  {
    std::cerr << "ERROR: "<< fileName << " must end with .pcd, .ply or .xyz" << std::endl;
  }
}

void DataConverter::read(std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // allocate memory if cloud is NULL
  if(cloud == NULL)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud2;
  }

  if      (fileName.find(".pcd") == fileName.size()-4)
  {
    pcl::io::loadPCDFile (fileName.c_str(), *cloud);
  }
  else if (fileName.find(".ply") == fileName.size()-4)
  {
    pcl::io::loadPLYFile (fileName.c_str(), *cloud);
  }
  else if (fileName.find(".xyz") == fileName.size()-4)
  {
    std::ifstream input(fileName.c_str());
    if(!input.is_open())
    {
      std::cerr << "ERROR: unable to open "<< fileName << std::endl;
      exit(-1);
    }
    std::string line;
    double x,y,z;
    while(getline(input,line))
    {
      input >> x >> y >> z;
      cloud->push_back(pcl::PointXYZ(x,y,z));
    }
    input.close();
  }
  else
  {
    std::cerr << "ERROR: "<< fileName << " must end with .pcd, .ply or .xyz" << std::endl;
  }
}


//pcl::PointCloud<pcl::PointXYZ>::Ptr DataConverter::getCloud()
//{
//  if(!empty)
//  {
//    return data;
//  }
//  else
//  {
//    std::cout << "WARNING [DataConverter::getCloudPointXYZ]: DataConverter is empty. No data loaded" << std::endl;
//  }
//}

void DataConverter::save(std::string fileName, std::vector< std::vector<double> > & vecMat)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  convert(vecMat,cloud);
  save(fileName,cloud);
}



void DataConverter::save(std::string fileName, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (fileName.find(".pcd")==fileName.size()-4)
  {
    pcl::io::savePCDFile (fileName.c_str(), cloud);
  }
  else if (fileName.find(".ply")==fileName.size()-4)
  {
    pcl::io::savePLYFile (fileName.c_str(), cloud);
  }
  else // otherwise, save as XYZ file
  {
    if(fileName.find(".xyz")!=fileName.size()-4)
    {
      fileName += ".xyz";
      std::cout << "WARNING [DataConverter::save]: unknown extension. Post processed data saved in " << fileName << std::endl;
    }
    
    std::ofstream out(fileName.c_str());
    if(!out.is_open())
    {
      std::cerr << "ERROR: unable to open " << fileName << std::endl;
      exit(-1);
    }
  
    for(size_t i=0;i<cloud.size();++i)
    {
  //    out << data[i].at<double>(0,0) << " " << data[i].at<double>(0,1) << " " << data[i].at<double>(0,2) << std::endl;
      out << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << std::endl;
    }
    
  }
}



void DataConverter::save(std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // allocate memory if cloud is NULL
  if(cloud == NULL)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud2;
  }
  
  if (fileName.find(".pcd")==fileName.size()-4)
  {
    pcl::io::savePCDFile (fileName.c_str(), *cloud);
  }
  else if (fileName.find(".ply")==fileName.size()-4)
  {
    pcl::io::savePLYFile (fileName.c_str(), *cloud);
  }
  else // otherwise, save as XYZ file
  {
    if(fileName.find(".xyz")!=fileName.size()-4)
    {
      fileName += ".xyz";
      std::cout << "WARNING [DataConverter::save]: unknown extension. Post processed data saved in " << fileName << std::endl;
    }
    
    std::ofstream out(fileName.c_str());
    if(!out.is_open())
    {
      std::cerr << "ERROR: unable to open " << fileName << std::endl;
      exit(-1);
    }
  
    for(size_t i=0;i<cloud->size();++i)
    {
  //    out << data[i].at<double>(0,0) << " " << data[i].at<double>(0,1) << " " << data[i].at<double>(0,2) << std::endl;
      out << (*cloud)[i].x << " " << (*cloud)[i].y << " " << (*cloud)[i].z << std::endl;
    }
  }
}


void DataConverter::saveAsOBJ (std::string &file_name, pcl::PolygonMesh &poly_mesh, unsigned precision = 6)
{
  if (poly_mesh.cloud.data.empty ())
  {
          PCL_ERROR ("[DataConverter::saveAsOBJ] Input point cloud is empty\n");
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  /* Write 3D information */
  // number of points
  int nr_points  = poly_mesh.cloud.width * poly_mesh.cloud.height;

  // mesh size
  int nr_meshes = poly_mesh.polygons.size();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " << nr_meshes << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(poly_mesh.cloud, *rgbCloud);
  
  for (unsigned int i=0; i<rgbCloud->points.size(); i++)
  {
          fs << "v " << rgbCloud->points[i].x << " " << rgbCloud->points[i].y << " " << rgbCloud->points[i].z << " 0 0 0"  << std::endl; //<< (double)((double)rgbCloud->points[i].r/255.0f) << " "  << (double)((double)rgbCloud->points[i].g/255.0f) << " " << (double)((double)rgbCloud->points[i].b/255.0f) << std::endl;
  }

  fs << "# "<< nr_points <<" vertices" << std::endl;

  for(int m = 0; m < nr_meshes; ++m){
          // Write faces with "f"
          fs << "f ";
          size_t j = 0;
          for (j = 0; j < poly_mesh.polygons[m].vertices.size () - 1; ++j)
                  fs << poly_mesh.polygons[m].vertices[j] +1 << " "; // vertex index in obj file format starting with 1
          fs << poly_mesh.polygons[m].vertices[j]+1 <<  std::endl;
  }
  fs << "# End of File";

  // Close obj file
  fs.close ();
//        return (0);
}

void DataConverter::saveAsSTL(std::string &file_name, pcl::PolygonMesh &poly_mesh, unsigned precision = 6)
{
  if (poly_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[DataConverter::saveAsSTL] Input point cloud is empty\n");
  }
  // Open file
  std::ofstream fs;
////  fs.precision (precision);
////  fs.open (file_name.c_str ());
  
  pcl::PointCloud<pcl::PointXYZRGB> vertices;
  pcl::fromROSMsg(poly_mesh.cloud, vertices);

//  fs << "solid DaMesh" << std::endl;
  std::cerr.precision (precision);
  std::cerr <<  poly_mesh.polygons[1].vertices[0] <<  std::endl;
  // For each polygon
////  for (size_t i=0; i<poly_mesh.polygons.size(); ++i)
////  {
////    fs << " facet normal " << 
//    
////    fs << "  facet normal " << XXXX << std::endl;
////    fs << "    outer loop" << std::endl;
////    for (size_t j=0; j<poly_mesh.polygons[i].vertices.size(); ++j)
////    {
////      fs << "      vertex " << poly_mesh.polygons[i].vertices[j] << std::endl;
////    }
////  } 
//  


//  fs << "endsolid DaMesh" << std::endl;

}


void DataConverter::savePolygon(std::string fileName, pcl::PolygonMesh & mesh)
{
  if (fileName.find(".vtk")==fileName.size()-4)
  {
    pcl::io::savePolygonFileVTK (fileName.c_str(), mesh);
  }
  else if (fileName.find(".ply")==fileName.size()-4)
  {
    pcl::io::savePolygonFilePLY (fileName.c_str(), mesh);
  }
  else if (fileName.find(".obj")==fileName.size()-4)
  {
    saveAsOBJ (fileName, mesh);
  }
  else // otherwise save as STL file (with the above function)
  {
    //saveAsSTL (fileName, mesh);
    pcl::io::savePolygonFileSTL(fileName.c_str(), mesh);
  }
}

void DataConverter::saveInXML(std::string fileName, std::vector<std::string> varNames, std::vector<cv::Mat> vars )
{
  cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
  if(varNames.size() == vars.size())
  {
    for(int i=0;i<vars.size();++i)
      fs << varNames[i] << vars[i];
  }
  fs.release();
}


