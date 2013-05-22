#include "PostProcessor.h"

PostProcessor::PostProcessor()
{
  std::cerr << "Warning: no name given for the input file." << std::endl << "Input and output file are set to \"data.gp\" and \"data_PP.gp\"." << std::endl;
  
  std::ifstream input("data.gp");
  if(!input.is_open())
  {
    std::cerr << "Error: unable to open \"data.gp\"" << std::endl;
    exit(-1);
  }
  else
  {
    inputFile = "data.gp";
  }
  
  input.close();
  
  std::ofstream output("data_PP.gp");
  if(!output.is_open())
  {
    std::cerr << "Error: unable to open \"data_PP.gp\"" << std::endl;
    exit(-1);
  }
  else
  {
    outputFile = "data_PP.gp";
  }
  output.close();
  
  
  read();
  
}


PostProcessor::PostProcessor(std::string inFile, std::string outFile)
{
  std::ifstream input(inFile.c_str());
  if(!input.is_open())
  {
    std::cerr << "Error: unable to open \""<< inFile << "\"" << std::endl;
    exit(-1);
  }
  else
  {
    inputFile = inFile;
  }
  input.close();
  
  std::ofstream output(outFile.c_str());
  if(!output.is_open())
  {
    std::cerr << "Error: unable to open \""<< outFile << "\"" << std::endl;
    exit(-1);
  }
  else
  {
    outputFile = outFile;
  }
  output.close();
  
  read(); 
}



PostProcessor::PostProcessor(std::vector<cv::Mat> & pData, std::string outFile)
{
  std::ofstream output(outFile.c_str());
  if(!output.is_open())
  {
    std::cerr << "Error: unable to open \""<< outFile << "\"" << std::endl;
    exit(-1);
  }
  else
  {
    outputFile = outFile;
  }
  output.close();
  data = pData;
}



void PostProcessor::read()
{
  std::ifstream input(inputFile.c_str());
  
  if(!input.is_open())
  {
    std::cerr << "Error: unable to open \""<< inputFile << "\"" << std::endl;
    exit(-1);
  }

  std::string line;
  cv::Mat point(1,3,cv::DataType<double>::type); // current point
  while(getline(input,line))
  {
    input >> point.at<double>(0,0) >> point.at<double>(0,1) >> point.at<double>(0,2);
    data.push_back(point);
  }
  input.close();
}


void PostProcessor::keepInCylinder(cv::Point3d center, double radius, double height)
{
  double x,y,z;
  double r2=radius*radius;
  for(unsigned i=0;i<data.size();++i)
  {
    x=data[i].at<double>(0,0);
    y=data[i].at<double>(0,1);
    z=data[i].at<double>(0,2);
    if(x*x + y*y <= r2 && center.z <= z && z <= center.z + height)
    {
      dataPP.push_back(data[i]);
    }
    
    // Progress bar
    std::cout << "[";//                \r" << (((double) 100*(i+1))/((double)data.size())) << "%\r";
    for(int k=1;k<=(int)(20*(i+1)/data.size());++k)
      std::cout << "|";
    for(int k=((int)(20*(i+1)/data.size())); k<20;++k)
      std::cout << " ";
    std::cout << "]\r";
  }
  std::cout << std::endl;
}




void PostProcessor::save()
{
//  double x,y,z;
  std::ofstream output(outputFile.c_str());
  if(!output.is_open())
  {
    std::cerr << "Error: unable to open \""<< outputFile << "\"" << std::endl;
    exit(-1);
  }
  
  for(size_t i=0;i<dataPP.size();++i)
  {
    output << dataPP[i].at<double>(0,0) << " " << dataPP[i].at<double>(0,1) << " " << dataPP[i].at<double>(0,2) << std::endl;
  }
}



