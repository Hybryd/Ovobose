#include "PostProcessor.h"

PostProcessor::PostProcessor()
{
//  std::cerr << "WARNING: no name given for the input file." << std::endl << "Input and output file are set to data/data.xyz and data/data_PP.xyz" << std::endl;
//  
//  std::ifstream input("data/data.xyz");
//  if(!input.is_open())
//  {
//    std::cerr << "ERROR: unable to open data/data.xyz" << std::endl;
//    exit(-1);
//  }
//  else
//  {
//    inputFile = "data/data.xyz";
//  }
//  
//  input.close();
//  
//  std::ofstream output("data/data_PP.xyz");
//  if(!output.is_open())
//  {
//    std::cerr << "ERROR: unable to open data/data_PP.xyz" << std::endl;
//    exit(-1);
//  }
//  else
//  {
//    outputFile = "data/data_PP.xyz";
//  }
//  output.close();
//  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr datatmp (new pcl::PointCloud<pcl::PointXYZ>);
//  data = datatmp;
//  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr dataPPtmp (new pcl::PointCloud<pcl::PointXYZ>);
//  dataPP = dataPPtmp;
//  
//  read();
//  
}


//PostProcessor::PostProcessor(std::string inFile, std::string outFile)
//{
//  std::ifstream input(inFile.c_str());
//  if(!input.is_open())
//  {
//    std::cerr << "ERROR: unable to open " << inFile << std::endl;
//    exit(-1);
//  }
//  else
//  {
//    inputFile = inFile;
//  }
//  input.close();
//  
//  std::ofstream output(outFile.c_str());
//  if(!output.is_open())
//  {
//    std::cerr << "ERROR: unable to open "<< outFile << std::endl;
//    exit(-1);
//  }
//  else
//  {
//    outputFile = outFile;
//  }
//  output.close();
//  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr datatmp (new pcl::PointCloud<pcl::PointXYZ>);
//  data = datatmp;
//  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr dataPPtmp (new pcl::PointCloud<pcl::PointXYZ>);
//  dataPP = dataPPtmp;
//  
//  
//  read(); 
//}



//PostProcessor::PostProcessor(pcl::PointCloud<pcl::PointXYZ>::Ptr pData, std::string outFile)
//{
//  std::ofstream output(outFile.c_str());
//  if(!output.is_open())
//  {
//    std::cerr << "ERROR: unable to open "<< outFile << std::endl;
//    exit(-1);
//  }
//  else
//  {
//    outputFile = outFile;
//  }
//  output.close();
//  data = pData;
//  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr dataPPtmp (new pcl::PointCloud<pcl::PointXYZ>);
//  dataPP = dataPPtmp;
//}



//void PostProcessor::read()
//{
//  std::ifstream input(inputFile.c_str());
//  
//  if(!input.is_open())
//  {
//    std::cerr << "ERROR: unable to open "<< inputFile << std::endl;
//    exit(-1);
//  }

//  std::string line;
//  double x,y,z;
//  cv::Mat point(1,3,cv::DataType<double>::type); // current point
//  while(getline(input,line))
//  {
//    input >> x >> y >> z;
//    data->push_back(pcl::PointXYZ(x,y,z));
//  }
//  input.close();
//}


void PostProcessor::smoothMesh(pcl::PolygonMesh::Ptr meshIn, pcl::PolygonMesh & meshOut, unsigned long int nbIter, double convergence, double relaxFactor, bool edgeSmoothing, bool angle, bool boundarySmoothing)
{
  std::cout << "Begin Laplacian VTKSmoothing...";
  pcl::PolygonMesh output;
  pcl::MeshSmoothingLaplacianVTK vtk;
  vtk.setInputMesh(meshIn);
//  vtk.setNumIter(20000);
//  vtk.setConvergence(0.0001);
//  vtk.setRelaxationFactor(0.0001);
//  vtk.setFeatureEdgeSmoothing(true);
//  vtk.setFeatureAngle(M_PI/5);
//  vtk.setBoundarySmoothing(true);
  vtk.setNumIter(nbIter);
  vtk.setConvergence(convergence);
  vtk.setRelaxationFactor(relaxFactor);
  vtk.setFeatureEdgeSmoothing(edgeSmoothing);
  vtk.setFeatureAngle(angle);
  vtk.setBoundarySmoothing(boundarySmoothing);
  vtk.process(meshOut);
  std::cout << "Done." << std::endl;
}

void PostProcessor::smoothCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, bool polygonalFit, double radius)
{
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointNormal> mls_points;

//  // Init object (second point type is for the normals, even if unused)
//  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
// 
//  mls.setComputeNormals (true);

//  // Set parameters
//  mls.setInputCloud (dataRaw);
//  mls.setPolynomialFit (true);
//  mls.setSearchMethod (tree);
//  mls.setSearchRadius (0.03);

//  // Reconstruct
//  mls.process (mls_points);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
 
  mls.setComputeNormals (false);

  // Set parameters
  mls.setInputCloud (dataRaw);
  mls.setPolynomialFit (polygonalFit);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (radius);

  // Reconstruct
  mls.process (*dataPP);

}

void PostProcessor::keepInCylinder(pcl::PointCloud<pcl::PointXYZ> & dataRaw, pcl::PointCloud<pcl::PointXYZ> & dataPP, pcl::PointXYZ center, double radius, double height)
{
  double x,y,z;
  double r2=radius*radius;
  for(unsigned i=0;i<dataRaw.size();++i)
  {
//    x=data[i].at<double>(0,0);
//    y=data[i].at<double>(0,1);
//    z=data[i].at<double>(0,2);
    x=dataRaw[i].x;
    y=dataRaw[i].y;
    z=dataRaw[i].z;
    if(x*x + y*y <= r2 && center.z <= z && z <= center.z + height)
    {
      dataPP.push_back(dataRaw[i]);
    }
    
    // Progress bar
    std::cout << "[";//                \r" << (((double) 100*(i+1))/((double)data.size())) << "%\r";
    for(int k=1;k<=(int)(20*(i+1)/dataRaw.size());++k)
      std::cout << "|";
    for(int k=((int)(20*(i+1)/dataRaw.size())); k<20;++k)
      std::cout << " ";
    std::cout << "]\r";
  }
  std::cout << std::endl;
  std::cout << "Ratio: " << dataPP.size() << "/" << dataRaw.size() << " (" << (100*(double)(dataRaw.size()-dataPP.size())/((double)dataRaw.size())) << " % won)" << std::endl;
}

void PostProcessor::keepInCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr dataRaw, pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP, pcl::PointXYZ center, double radius, double height)
{
  // check if dataRaw is not NULL
  if(dataRaw == NULL)
  {
    std::cerr << "WARNING [PostProcessor::keepInCylinder]: dataRaw is NULL" << std::endl;
    exit(-1);
  }

  // allocate memory if dataPP is NULL
  if(dataPP == NULL)
  {
    std::cout << "WARNING [PostProcessor::keepInCylinder]: dataPP was not allocated" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dataPP2 (new pcl::PointCloud<pcl::PointXYZ>);
    dataPP = dataPP2;
  }
    
  keepInCylinder(*dataRaw, *dataPP, center, radius, height);

//  double x,y,z;
//  double r2=radius*radius;
//  for(unsigned i=0;i<dataRaw->size();++i)
//  {
////    x=data[i].at<double>(0,0);
////    y=data[i].at<double>(0,1);
////    z=data[i].at<double>(0,2);
//    x=(*dataRaw)[i].x;
//    y=(*dataRaw)[i].y;
//    z=(*dataRaw)[i].z;
//    if(x*x + y*y <= r2 && center.z <= z && z <= center.z + height)
//    {
//      dataPP->push_back((*dataRaw)[i]);
//    }
//    
//    // Progress bar
//    std::cout << "[";//                \r" << (((double) 100*(i+1))/((double)data.size())) << "%\r";
//    for(int k=1;k<=(int)(20*(i+1)/dataRaw->size());++k)
//      std::cout << "|";
//    for(int k=((int)(20*(i+1)/dataRaw->size())); k<20;++k)
//      std::cout << " ";
//    std::cout << "]\r";
//  }
//  std::cout << std::endl;
//  std::cout << "Ratio: " << dataPP->size() << "/" << dataRaw->size() << " (" << (100*(dataRaw->size()-dataPP->size())/dataRaw->size()) << " % won)" << std::endl;
}



