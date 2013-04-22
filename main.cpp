#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <cv.h>
#include <highgui.h>

// Global parameters
int V_cut=200;
int H_cut=100;

void onTrackbar(int, void* = NULL)
{
}

using namespace cv;
int main(int argc, char ** argv)
{
  
  CvCapture * capture = NULL; // capture
  IplImage * image = NULL; // capture assignation
  cv::Mat image2; // clone of image
  cv::Mat image2HSV; // clone of image HSV version
  cv::Mat image2HSV2; // clone of image2HSV
  cv::Mat image2HSVGRAY; // clone of image2HSV
  
  cv::Mat img; // image for the calibration
  cv::Mat imgGRAY; // image for the calibration GRAY
  cv::Mat imgGRAY2; // image for the calibration GRAY
  cv::Mat imgWL; // image withlines
  
  vector<Vec2f> lines; // contains the set of lines found by HoughLines
  Point ptX;
  Point pt0;
  Point ptZ;
  
  double aVert,bVert,aDiag,bDiag;
  

	capture = cvCreateCameraCapture( 0 );
	assert(capture >= 0 && "Can not initialize video capture.");

	cvNamedWindow( "Real image" );
	cvNamedWindow( "Thresholded GRAY" );
	cvNamedWindow( "Thresholded GRAY with lines" );
	
	image = cvQueryFrame( capture );
	
	
	cv::createTrackbar("Threshold", "Thresholded GRAY", &V_cut, 250, onTrackbar);
  onTrackbar(0);
	cv::createTrackbar("Hough", "Thresholded GRAY with lines", &H_cut, 200, onTrackbar);
  onTrackbar(0);

	image2=image;
	std::cerr << "Size of the image: " << image2.rows << " " << image2.cols  << std::endl;
	
	
	
	//////////////////////////////
	// Wait for detecting lines //
	//////////////////////////////
	
	int c=0;
	while( image && c != 32 ) //spacebar to take image
	{
	  image2 = image;
	  cv::cvtColor(image2, image2HSV, CV_BGR2HSV);
	  image2HSV2 = image2HSV;
	  
	  // threshold image2HSV2
	  for(int i=0; i<image2HSV2.rows; i++)
    {
      for(int j=0; j<image2HSV2.cols; j++)
      {
        if(image2HSV.at<cv::Vec3b>(i,j)[2] < V_cut)
        {
          image2HSV2.at<cv::Vec3b>(i,j)[0] = 0;
          image2HSV2.at<cv::Vec3b>(i,j)[1] = 0;
          image2HSV2.at<cv::Vec3b>(i,j)[2] = 0;
        }
        else
        {
          image2HSV2.at<cv::Vec3b>(i,j) = image2HSV.at<cv::Vec3b>(i,j);
        }
      }
    }
    cv::blur( image2HSV2, image2HSV2, cv::Size(3,3) );
	  cvtColor( image2HSV2, image2HSVGRAY, CV_BGR2GRAY );
	  
	  imshow( "Real image", image2 );
	  cv::putText(image2, "Press 'Space' to take the snapshot", cvPoint(200,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.45, cvScalar(200,200,200), 1);
	  
	  c = cvWaitKey( 15 );
	  
    img = image2;
    imgGRAY = image2HSVGRAY;
	  
    imshow("Thresholded GRAY", imgGRAY);
  
    // Find middle of laser
    imgGRAY2 = imgGRAY.clone();
    imgGRAY2 = Scalar(0);
    std::vector<int> row;
    for(int i=0; i<imgGRAY.rows; i++)
    {
      for(int j=0; j<imgGRAY.cols; j++)
      {
        if(imgGRAY.at<unsigned char>(i,j) != 0)
          row.push_back(j);
      }
//        std::cerr << row.size() << std::endl;
      if(row.size()!=0)
        imgGRAY2.at<unsigned char>(i,row[row.size()/2]) = 250;
      row.clear();
    }
    
    // Find lines
    
    //	Canny( imgGRAY2, imgGRAY2, 100, 200, 3 );
    
    //c=0;
    imgWL = img.clone();
   // while(c!='q')
   //{
    HoughLines( imgGRAY2, lines, 1, CV_PI/180, H_cut );
    
    // Collect the pairs of points defining the lines
    std::vector< std::pair<Point, Point> > setPts;
    for( size_t i = 0; i < lines.size(); i++ )
    {
      float rho = lines[i][0];
      float theta = lines[i][1];
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
      Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));
      std::pair<Point, Point> pair;
      pair.first = pt1;
      pair.second = pt2;
//        std::cerr << pt1 << " " << pt2 << std::endl;
      setPts.push_back(pair);
//        line( imgWL, pt1, pt2, Scalar(0,0,255), 1, CV_AA );
    }
//       std::cerr << "--------------------------" << std::endl;

    // 


    // Separate the two sets of lines according to their slope
    std::vector< std::pair< Point, Point> > vertLines;
    std::vector< std::pair< Point, Point> > diagLines;
    for(size_t i = 0;i<setPts.size();++i)
    {
      if((setPts[i].first.x-setPts[i].second.x) != 0)
      {
        if(abs((setPts[i].first.y-setPts[i].second.y)/(setPts[i].first.x-setPts[i].second.x)) > 30 ) // 30 is arbitrary
        {
          vertLines.push_back(setPts[i]);
        }
        else
        {
          diagLines.push_back(setPts[i]);
        }
      }
      else
      {
        vertLines.push_back(setPts[i]);
      }
    }




    // Average the two sets of lines to get 2 lines
    // check also that lines are defined by points in the same direction // TODO : swap if necessary ????
    double avgVertX1=0;
    double avgVertY1=0;
    double avgVertX2=0;
    double avgVertY2=0;
    double avgDiagX1=0;
    double avgDiagY1=0;
    double avgDiagX2=0;
    double avgDiagY2=0;
    
    for(size_t i = 0;i<vertLines.size();++i)
    {
      
      if(vertLines[i].first.y < vertLines[i].second.y)
      {
        avgVertX1 += vertLines[i].first.x;
        avgVertY1 += vertLines[i].first.y;
        avgVertX2 += vertLines[i].second.x;
        avgVertY2 += vertLines[i].second.y;
      }
      else
      {
        avgVertX1 += vertLines[i].second.x;
        avgVertY1 += vertLines[i].second.y;
        avgVertX2 += vertLines[i].second.x;
        avgVertY2 += vertLines[i].second.y;
      }
    }
    avgVertX1 /= vertLines.size();
    avgVertY1 /= vertLines.size();
    avgVertX2 /= vertLines.size();
    avgVertY2 /= vertLines.size();
    
    Point ptVert1(avgVertX1,avgVertY1);
    Point ptVert2(avgVertX2,avgVertY2);
    
    
    
    avgVertX1=0;
    avgVertY1=0;
    avgVertX2=0;
    avgVertY2=0;
    avgDiagX1=0;
    avgDiagY1=0;
    avgDiagX2=0;
    avgDiagY2=0;
    
    for(size_t i = 0;i<diagLines.size();++i)
    {
      
      if(diagLines[i].first.y < diagLines[i].second.y)
      {
        avgDiagX1 += diagLines[i].first.x;
        avgDiagY1 += diagLines[i].first.y;
        avgDiagX2 += diagLines[i].second.x;
        avgDiagY2 += diagLines[i].second.y;
      }
      else
      {
        avgDiagX1 += diagLines[i].second.x;
        avgDiagY1 += diagLines[i].second.y;
        avgDiagX2 += diagLines[i].second.x;
        avgDiagY2 += diagLines[i].second.y;
      }
    }
    avgDiagX1 /= diagLines.size();
    avgDiagY1 /= diagLines.size();
    avgDiagX2 /= diagLines.size();
    avgDiagY2 /= diagLines.size();
    
    Point ptDiag1(avgDiagX1,avgDiagY1);
    Point ptDiag2(avgDiagX2,avgDiagY2);
    
    
    
    // Find corresponding segments in the image. Equations: ax+b=y
    //double aVert;
    if (ptVert1.x==ptVert2.x)
      aVert=999999999999;
    else
      aVert = ((double)ptVert1.y-(double)ptVert2.y)/((double)ptVert1.x-(double)ptVert2.x);
    //double bVert = (double)ptVert1.y-(double)aVert*(double)ptVert1.x;
    bVert = (double)ptVert1.y-(double)aVert*(double)ptVert1.x;
    
//      std::cerr << "a,b vert " << aVert << " " << bVert << std::endl;
    
//    double aDiag = ((double)ptDiag1.y-(double)ptDiag2.y)/((double)ptDiag1.x-(double)ptDiag2.x);
//    double bDiag = (double)ptDiag1.y-(double)aDiag*(double)ptDiag1.x;
    aDiag = ((double)ptDiag1.y-(double)ptDiag2.y)/((double)ptDiag1.x-(double)ptDiag2.x);
    bDiag = (double)ptDiag1.y-(double)aDiag*(double)ptDiag1.x;
    
//      std::cerr << "a,b diag " << aDiag << " " << bDiag << std::endl;
    
    // coordinates of the intersection of the two lines
    double xInter= -(bVert-bDiag)/(aVert-aDiag);
    double yInter= aDiag*xInter + bDiag;
    Point ptInter(xInter, yInter);
    
//      std::cerr << ptInter << std::endl;

    // /!\ WARNING : DIAG MUST CROSS THE TOP OF THE IMAGE      
    ptVert1.y = 0;
    if(aVert!=0)
      ptVert1.x = -bVert/aVert;
    ptVert2 = ptInter;
    
    ptDiag1.y = img.rows;
    ptDiag1.x = (img.rows-bDiag)/aDiag;
    ptDiag2 = ptInter;
    
//      std::cerr << ptVert1 << " " << ptVert2 << std::endl;
//      std::cerr << ptDiag1 << " " << ptDiag2 << std::endl;
//      std::cerr << "--------------" << std::endl;
//            
    Point adel(xInter+100, yInter);
    line( imgWL, ptVert1, ptVert2, Scalar(0,0,255), 1, CV_AA );
    line( imgWL, ptDiag1, ptDiag2, Scalar(0,0,255), 1, CV_AA );
    
    //line( imgWL, ptInter, adel, Scalar(255,0,0), 1, CV_AA );
    
    lines.clear();
    cv::putText(imgWL, "Press 'q' to quit", cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.45, cvScalar(200,200,200), 1);
    imshow("Thresholded GRAY with lines", imgWL);
    imgWL = img.clone();
      //c = cvWaitKey( 15 );
    //}
  
  
  
	  ptX=ptDiag1;
	  pt0=ptInter;
	  ptZ=ptVert1;

	  image = cvQueryFrame( capture );
	}
	cvDestroyWindow("Real image");
	cvDestroyWindow("Thresholded GRAY");
	cvDestroyWindow("Thresholded GRAY with lines");
	
	
	
	
	/////////////////
	// Calibration //
	/////////////////
	
	CvPoint3D32f realZ;
	realZ.x=0;
	realZ.y=0;
	realZ.z=30.5;
	
	CvPoint3D32f real0;
	real0.x=0;
	real0.y=0;
	real0.z=0;
	
	CvPoint3D32f realX;
	realX.x=37.3;
	realX.y=0;
	realX.z=0;
	
	double stepZ=(realZ.z-real0.z)/(pt0.y-ptZ.y);
	double stepX=(realX.x-real0.x)/(ptX.y-pt0.y);
	
	// Print the matrix, octave style
	
	// Along the vertical line
	for(int y=ptZ.y; y<=pt0.y; ++y)
	{
	  // /!\ WARNING si verticale parfaite
    std::cout << y << " " << (y-bVert)/aVert << " " << 1 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << ";" << std::endl;
    std::cout << 0 << " " << 0 << " " << 0 << " " << y << " " << (y-bVert)/aVert << " " << 1 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << ";" << std::endl;
    std::cout << 0 << " " << 0 << " " << 0  << " " << 0 << " " << 0 << " " << 0 << " " << y << " " << (y-bVert)/aVert << " " << 1<< " " << -y*(realZ.z-stepZ) << " " << -(y-bVert)/aVert*(realZ.z-stepZ) << " " << -(realZ.z-stepZ) << ";" << std::endl;
	}

  // Along the diagonal line
	for(int y=pt0.y; y<=ptX.y; ++y)
	{
	  // /!\ WARNING si verticale parfaite
    std::cout << y << " " << (y-bDiag)/aDiag << " " << 1 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << -y*(real0.x+stepX) << " " << -(y-bDiag)/aDiag*(real0.x+stepX) << " " << -(real0.x+stepX) << ";" << std::endl;
    
    // test X=Y
    std::cout << 0 << " " << 0 << " " << 0 << " " << y << " " << (y-bVert)/aVert << " " << 1 << " " << 0 << " " << 0 << " " << 0 << " " << -y*(real0.x+stepX) << " " << -(y-bDiag)/aDiag*(real0.x+stepX) << " " << -(real0.x+stepX) << ";" << std::endl;
    
    std::cout << 0 << " " << 0 << " " << 0  << " " << 0 << " " << 0 << " " << 0 << " " << y << " " << (y-bVert)/aVert << " " << 1<< " " << 0 << " " << 0 << " " << 0 << ";" << std::endl;
	}



	return 0;
}

