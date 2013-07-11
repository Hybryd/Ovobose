#include "MainWin.h"
 
MainWin::MainWin()
{
  
  setWindowTitle(tr("Ovobose"));
//  QMenu *menuFile = menuBar()->addMenu("&File");
  
  // Open
//  QMenu *menuOpenFile = menuBar()->addMenu("&Open");
  actionOpenParamFile = new QAction(tr("&Open"), this);
//  menuFile->addAction(actionOpenParamFile);
  connect(actionOpenParamFile, SIGNAL(triggered()), this, SLOT(openParamFile()));
  actionOpenParamFile->setShortcut(QKeySequence(tr("Ctrl+O")));

  // Save
//  QMenu *menuSaveFile = menuBar()->addMenu("&Save");
  actionSaveFile = new QAction(tr("&Save"), this);
//  menuFile->addAction(actionSaveFile);
  connect(actionSaveFile, SIGNAL(triggered()), this, SLOT(saveFile()));
  actionSaveFile->setShortcut(QKeySequence(tr("Ctrl+S")));
    
  // Quit
  actionQuit = new QAction(tr("&Quit"), this);
//  menuFile->addAction(actionQuit);
  connect(actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));
  actionQuit->setShortcut(QKeySequence(tr("Ctrl+Q")));  
  
  
  
  
  widCam = new QOpenCVWidget(this);
  labCam = new QLabel(this);
  camera = cv::VideoCapture(0);
  camera.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH_IMAGE );
  camera.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT_IMAGE );
  
  // Mouse event from QOpenCVWidget
  connect(widCam, SIGNAL(mouseMoveEvent(QMouseEvent * e)), this, SLOT(mouseOverImage(QMouseEvent * e)));
  
  
  imgCam = cv::Mat(WIDTH_IMAGE,HEIGHT_IMAGE, cv::DataType<cv::Vec3b>::type );
  if(!camera.isOpened())
  {
    
//    for(int x=0; x<imgCam.cols; ++x)
//    {
//      for(int y=0; y<imgCam.rows; ++y)
//      {
//        imgCam.at<cv::Vec3b>(x,y)[0]=0;
//        imgCam.at<cv::Vec3b>(x,y)[1]=0;
//        imgCam.at<cv::Vec3b>(x,y)[2]=0;
//      }
//    }
    widCam->putTextOnly(QString("No camera detected"));
  }
  else
  {
    camera >> imgCam;
    widCam->putImage(imgCam);
  }
  
  widCam->resize(imgCam.cols+2*BORDER,imgCam.rows+2*BORDER);
  
  
  
  
  streamCpt = startTimer(100);
  
  
  
  
  
  tabs = new QTabWidget(this);
  tabs->setGeometry(BORDER,BORDER,widCam->width()+2*BORDER,widCam->height()+2*BORDER + HEIGHT_PROGRESS+100);
  tabs->setFixedHeight(widCam->height()+2*BORDER + HEIGHT_PROGRESS+100);
  
    pageCal = new QWidget;
      vbox1 = new QVBoxLayout;
      
        groupTop = new QGroupBox;

          hbox1 = new QHBoxLayout;
          
            // Group of buttons
            groupBut = new QGroupBox;
              
              vbox2 = new QVBoxLayout;
                // Button open parameters file
                butOpenParamFile = new QPushButton(tr("Open calibration"),this);
                connect(butOpenParamFile, SIGNAL(clicked()), this, SLOT(openParamFile()));
                
                // Button calibration
                butCalibration = new QPushButton("Calibrate",this);
                connect(butCalibration, SIGNAL(clicked()), this, SLOT(calibrate()));
                
              vbox2->addWidget(butOpenParamFile);
              vbox2->addWidget(butCalibration);
              
            groupBut->setLayout(vbox2);
          
          
            // Matrix
            groupMat = new QGroupBox(tr("Transformation matrix"));
            groupMat->setAlignment(Qt::AlignHCenter);
        
              
              gridMat = new QGridLayout;
              for(int i=0;i<3;++i)
              {
                std::vector< QLabel * > row;
                for(int j=0;j<4;++j)
                {
                  row.push_back(new QLabel("0",this));
                }
                labMat.push_back(row);
              }
              for(int i=0;i<3;++i)
              {
                for(int j=0;j<4;++j)
                {
                  gridMat->addWidget(labMat[i][j],i,j);
                }
              }
              
            groupMat->setLayout(gridMat);
          
            // Normal vector
            groupVec = new QGroupBox(tr("Normal vector"));
              
              gridVec = new QGridLayout;
              gridVec->setAlignment(Qt::AlignHCenter);
                for(int i=0;i<3;++i)
                {
                  labVec.push_back(new QLabel("0",this));
                }
              for(int i=0;i<3;++i)
              {
                gridVec->addWidget(labVec[i],i,0);
              }
              
            groupVec->setLayout(gridVec);
          
          
          hbox1->addWidget(groupBut);  
          hbox1->addWidget(groupMat);
          hbox1->addWidget(groupVec);
        
        groupTop->setLayout(hbox1);
      
        // Calibration Group box
        groupCal = new QGroupBox;
          hboxCal = new QHBoxLayout;
            
            labCalNbPoints = new QLabel("Number of points",this);
            sbCalNbPoints = new QSpinBox(this);
            sbCalNbPoints->setRange(10,100);
            
            labCalRad = new QLabel("Radius",this);
            sbCalRad = new QSpinBox(this);
            sbCalRad->setRange(0,100);
            
            labCalAngle = new QLabel("Angular step",this);
            sbCalAngle = new QSpinBox(this);
            sbCalAngle->setRange(0,10);
            
          hboxCal->addWidget(labCalRad); 
          hboxCal->addWidget(sbCalRad);
          hboxCal->addWidget(labCalNbPoints);
          hboxCal->addWidget(sbCalNbPoints);
          hboxCal->addWidget(labCalAngle);
          hboxCal->addWidget(sbCalAngle);
        groupCal->setLayout(hboxCal);
        
      vbox1->addWidget(groupTop);
      vbox1->addWidget(groupCal);
      vbox1->addWidget(widCam);
      
      
      
    pageCal->setLayout(vbox1);

    pageScan = new QWidget;

  
  tabs->addTab(pageCal, "Calibration");
  tabs->setTabEnabled(0,true);
  tabs->addTab(pageScan, "Scan");
  tabs->setTabEnabled(1,false);
  
  tabs->addAction(actionOpenParamFile);
  tabs->addAction(actionSaveFile);
  tabs->addAction(actionQuit);
  
  
  progress = new QProgressBar(this);
  progress->setVisible(false);
  progress->setMaximumHeight(HEIGHT_PROGRESS);
  statusBar()->addWidget(progress, 1);
  statusBar()->showMessage(tr("Waiting for parameters"));
  
  
  loadDefaultParameters();
  resize(tabs->width() + 2*BORDER,tabs->height()+2*BORDER);
  
  calibration=false;
}


void MainWin::loadDefaultParameters()
{
  param_file                           = "";//"param/cal_param.xml";
  cal_circular_pattern_radius          = 0; //5.6;
  cal_circular_pattern_number_points   = 0; //20;
//  param_name_matrix                    = "matM";
//  param_name_normal                    = "vecN";
  scan_output_file                     = "";//"data/scan_output.xyz";
  scan_step_angle                      = 0; //1;
}


bool MainWin::checkIfCamera()
{
  cv::VideoCapture capture = cv::VideoCapture(0);
  return(capture.isOpened());
}



void MainWin::openParamFile()
{

  QString filename = QFileDialog::getOpenFileName( 
                                                    this, 
                                                    tr("Open"), 
                                                    QDir::currentPath(), 
                                                    tr("Parameter files (*.xml)")
                                                 );
                                                 
  
  if(filename != "")
  {
                                                   
    std::string lM = "matM";
    std::string lN = "vecN";
    
  
    cv::FileStorage fs(filename.toUtf8().constData(), cv::FileStorage::READ); // parameters file
    
    matM.create(3,4,cv::DataType<double>::type);
    vecN.create(4,1,cv::DataType<double>::type);
    
    fs[lM] >> matM;
    fs[lN] >> vecN;
    
    // upate display of matrix
    
    for(int i=0;i<3;++i)
    {
      for(int j=0;j<4;++j)
      {
        QString s("");
        QString number(QString::number(matM.at<double>(i,j)));
        int k=0;
        while (k<number .size() && k<4)
        {
          s += number[k];
          ++k;
        }
        labMat[i][j]->setAlignment(Qt::AlignCenter);
        labMat[i][j]->setText(s);
      }
    }
    
    for(int i=0;i<3;++i)
    {
      labVec[i]->setAlignment(Qt::AlignCenter);
      labVec[i]->setText(QString::number(vecN.at<double>(i,0)));
    }
                                                     
  //   // Read parameters
  //  std::string line;
  //  std::string keyword;
  //  std::stringstream sline;
  //  std::cout << "IN OPEN " << filename.toUtf8().constData() << std::endl;
  //  std::fstream file(filename.toUtf8().constData(),std::ios::in);
  //  if(file)
  //  {
  //    while(getline(file,line))
  //    {
  //      if(line != "") // avoids empty lines
  //      {
  //        sline.clear();
  //        sline.str("");
  //        sline << line;
  //        sline >> keyword;
  //        // Parser
  //        if(keyword[0] == '#')
  //        {
  //          if      (keyword == "#param_file")
  //          {
  //            sline >> param_file;
  //          }
  ////          else if (keyword == "#cal_needed")
  ////          {
  ////            sline >> cal_needed;
  ////          }
  //          else if (keyword == "#cal_circular_pattern_radius")
  //          {
  //            sline >> cal_circular_pattern_radius;
  //          }
  //          else if (keyword == "#cal_circular_pattern_number_points")
  //          {
  //            sline >> cal_circular_pattern_number_points;
  //          }
  ////          else if (keyword == "#param_name_matrix")
  ////          {
  ////            sline >> param_name_matrix;
  ////          }
  ////          else if (keyword == "#param_name_normal")
  ////          {
  ////            sline >> param_name_normal;
  ////          }
  //          else if (keyword == "#scan_output_file")
  //          {
  //            sline >> scan_output_file;
  //          }
  //          else if (keyword == "#scan_step_angle")
  //          {
  //            sline >> scan_step_angle;
  //          }
  ////            else if (keyword == "#post_proc_output_file")
  ////            {
  ////              sline >> post_proc_output_file;
  ////            }
  ////            else if (keyword == "#post_proc_cylinder_center")
  ////            {
  ////              double x,y,z;
  ////              sline >> x >> y >> z;
  ////              post_proc_cylinder_center = pcl::PointXYZ(x,y,z);
  ////            }
  ////            else if (keyword == "#post_proc_cylinder_radius")
  ////            {
  ////              sline >> post_proc_cylinder_radius;
  ////            }
  ////            else if (keyword == "#post_proc_cylinder_height")
  ////            {
  ////              sline >> post_proc_cylinder_height;
  ////            }
  //          else
  //          {
  //          
  //          }
  //        }
  //      }
  //    }
  //  }
  //  else
  //  {
  //    std::cout << "unable to open file" << std::endl;
  //  }
  //  std::cout << " 1) Parameters: " << std::endl;
  //  std::cout << "     param_file                           " << param_file << std::endl;
  ////  std::cout << "     cal_needed                           " << cal_needed << std::endl;
  //  std::cout << "     cal_circular_pattern_radius          " << cal_circular_pattern_radius << std::endl;
  //  std::cout << "     cal_circular_pattern_number_points   " << cal_circular_pattern_number_points << std::endl;
  ////  std::cout << "     param_name_matrix                    " << param_name_matrix << std::endl;
  ////  std::cout << "     param_name_normal                    " << param_name_normal << std::endl;
  //  std::cout << "     scan_output_file                     " << scan_output_file << std::endl;
  //  std::cout << "     scan_step_angle                      " << scan_step_angle << std::endl;
  //  
    

    if(checkIfParamOk())
    {
      tabs->setTabEnabled(1,true);
      statusBar()->showMessage(tr("Scan ready"));
    }
  }
}


// True if there exists at least one non zero element in matM and vecN 
bool MainWin::checkIfParamOk()
{
  bool res=false;
  for(int i=0;i<3;++i)
  {
    for(int j=0;j<4;++j)
    {
      res |= (matM.at<double>(i,j) != 0);
    }
    res |= (vecN.at<double>(i,0) != 0);
  }
  return res;
}


void MainWin::calibrate()
{
  if(!checkIfCamera())
  {
    statusBar()->showMessage(tr("No camera detected"), 5000);
  }
  else
  {
//    Calibration cal(param_file);
    Calibration cal;
//    cal.circularPattern(cal_circular_pattern_number_points,cal_circular_pattern_radius);
    cal.circularPattern(20,5.6);
    matM = cal.getMat();
    vecN = cal.getVec();
    
    
    // update display (put in a function)
    for(int i=0;i<3;++i)
    {
      for(int j=0;j<4;++j)
      {
        QString s("");
        QString number(QString::number(matM.at<double>(i,j)));
        int k=0;
        while (k<number.size() && k<4)
        {
          s += number[k];
          ++k;
        }
        labMat[i][j]->setAlignment(Qt::AlignCenter);
        labMat[i][j]->setText(s);
      }
    }
    
    for(int i=0;i<3;++i)
    {
      labVec[i]->setAlignment(Qt::AlignCenter);
      labVec[i]->setText(QString::number(vecN.at<double>(i,0)));
    }
    
    

    
//    cal.save("matM","vecN");
    if(checkIfParamOk())
    {
      tabs->setTabEnabled(1,true);
      statusBar()->showMessage(tr("Scan ready"));
    }
  }
}

void MainWin::saveCalibrationParam()
{
  QString filename = QFileDialog::getSaveFileName(  
                                                    this, 
                                                    tr("Save calibration parameters"), 
                                                    QDir::currentPath(), 
                                                    tr("All files (*.xml)")
                                                 );
  if(filename != "")
  {

    DataConverter dc;
    std::vector<std::string> varNames;
    std::vector<cv::Mat> vars;
    
    varNames.push_back("matM");
    varNames.push_back("vecN");
    
    vars.push_back(matM);
    vars.push_back(vecN);
    dc.saveInXML(filename.toUtf8().constData(), varNames, vars);
  }
  else
  {
    std::cerr << "ERROR in MainWin::saveCalibrationParam : wrong file name." << std::endl;
  }
}


void MainWin::saveFile()
{

  QString filename = QFileDialog::getSaveFileName(  
                                                    this, 
                                                    tr("Save"), 
                                                    QDir::currentPath(), 
                                                    tr("All files (*.*)")
                                                 );
}


void MainWin::timerEvent(QTimerEvent* e)
{
	if(e->timerId()==streamCpt)
	{
	  if(camera.isOpened())
	  {
		  camera >> imgCam;
		  widCam->putImage(imgCam);
		}
		else
		{
		  camera = cv::VideoCapture(0);
		  widCam->putTextOnly(QString("No camera detected"));
		}
	}
}


void MainWin::mousePressEvent(QMouseEvent *event)
{
//  if (event->button() == Qt::LeftButton)
//  {
//    lastPoint = event->pos();
//  }
}

void MainWin::mouseMoveEvent(QMouseEvent *event)
{
  
//  if ((event->buttons() & Qt::LeftButton))
//  {
//    statusBar()->showMessage(QString("Position: ") + QString::number(event->pos().x()) + QString("/") + QString::number(event->pos().y()));
//    std::cerr <<  event->pos().x() << " " << event->pos().y()  << std::endl;
//  }
}

void MainWin::mouseReleaseEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton && calibration)
  {
    std::pair< int,int > p;
    p.first = event->pos().x();
    p.second = event->pos().y();
    vecCalPoints.push_back(p);
  }
}

void MainWin::resizeEvent(QResizeEvent *event)
{
//     if (width() > image.width() || height() > image.height()) {
//         int newWidth = qMax(width() + 128, image.width());
//         int newHeight = qMax(height() + 128, image.height());
//         resizeImage(&image, QSize(newWidth, newHeight));
//         update();
//     }
//     QWidget::resizeEvent(event);
}

void MainWin::newScan()
{
  


}




void MainWin::mouseOverImage(QMouseEvent * e)
{
  statusBar()->showMessage(QString("Position: ") + QString::number(e->pos().x()) + QString("/") + QString::number(e->pos().y()));
  std::cerr <<  e->pos().x() << " " << e->pos().y()  << std::endl;
}


