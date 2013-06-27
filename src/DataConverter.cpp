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


