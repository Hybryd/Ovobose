#include "MainWin.h"
 
MainWin::MainWin()
{
  resize(600,400);
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
  
  
//  // Central widget
//  QWidget *central = new QWidget;
//  setCentralWidget(central);
//  QFormLayout *layout = new QFormLayout;

//  // Button calibration
//  QPushButton *butCalibration = new QPushButton("Calibrate",this);
//  QAction *actionCalibration = new QAction("&Calibration", this);
//  connect(actionCalibration, SIGNAL(triggered()), this, SLOT(calibrate()));
//  layout->addRow(butCalibration);
//  
//  // Labels for parameters
////  QLabel *lab = new QLabel(tr("Parameters for calibration"), this);


//  central->setLayout(layout);
  
  tabs = new QTabWidget(this);
  tabs->setGeometry(5,5,this->width()-10,this->height()-10);
  
    pageCal = new QWidget;
      vbox1 = new QVBoxLayout;
      
        groupTop = new QGroupBox;

          hbox1 = new QHBoxLayout;
          
            // Group of buttons
            groupBut = new QGroupBox;
              
              vbox2 = new QVBoxLayout;
              
                // Button calibration
                butCalibration = new QPushButton("Calibrate",this);
                connect(butCalibration, SIGNAL(clicked()), this, SLOT(calibrate()));
                
                // Button open parameters file
                butOpenParamFile = new QPushButton(tr("Open parameter file"),this);
                connect(butOpenParamFile, SIGNAL(clicked()), this, SLOT(openParamFile()));
              vbox2->addWidget(butCalibration);
              vbox2->addWidget(butOpenParamFile);
              
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
              
//                QLabel * mat00 = new QLabel("0",this);  QLabel * mat01 = new QLabel("0",this); QLabel * mat02 = new QLabel("0",this);
//                QLabel * mat10 = new QLabel("0",this);  QLabel * mat11 = new QLabel("0",this); QLabel * mat12 = new QLabel("0",this);
//                QLabel * mat20 = new QLabel("0",this);  QLabel * mat21 = new QLabel("0",this); QLabel * mat22 = new QLabel("0",this);
//              gridMat->addWidget(mat00,0,0); gridMat->addWidget(mat01,0,1); gridMat->addWidget(mat02,0,2);
//              gridMat->addWidget(mat10,1,0); gridMat->addWidget(mat11,1,1); gridMat->addWidget(mat12,1,2);
//              gridMat->addWidget(mat20,2,0); gridMat->addWidget(mat21,2,1); gridMat->addWidget(mat22,2,2);
//              vbox2->addWidget(butOpenParamFile);
              
            groupMat->setLayout(gridMat);
          
            // Normal vector
            groupVec = new QGroupBox(tr("Normal vector"));
            groupVec->setAlignment(Qt::AlignHCenter);
        
              
              gridVec = new QGridLayout;
                for(int i=0;i<3;++i)
                {
                  labVec.push_back(new QLabel("0",this));
                }
              for(int i=0;i<3;++i)
              {
                gridVec->addWidget(labVec[i],i,0);
              }
//                QLabel * vec00 = new QLabel("0",this);
//                QLabel * vec10 = new QLabel("0",this);
//                QLabel * vec20 = new QLabel("0",this);
//              gridVec->addWidget(vec00,0,0);
//              gridVec->addWidget(vec10,1,0);
//              gridVec->addWidget(vec20,2,0);
//              vbox2->addWidget(butOpenParamFile);
              
            groupVec->setLayout(gridVec);
          
          
          hbox1->addWidget(groupBut);  
          hbox1->addWidget(groupMat);
          hbox1->addWidget(groupVec);
          
        
        
        
        groupTop->setLayout(hbox1);



        labBot = new QLabel("WEBCAM ICI");
      
      
      
      
      
//        QHBoxLayout *hbox1 = new QHBoxLayout;
//          
//          QVBoxLayout *vbox2 = new QVBoxLayout;
//          
//            // Button calibration
//            QPushButton *butCalibration = new QPushButton("Calibrate",this);
//            connect(butCalibration, SIGNAL(clicked()), this, SLOT(calibrate()));
//            
//            // Button open parameters file
//            QPushButton *butOpenParamFile = new QPushButton("Open parameter file",this);
//            connect(butOpenParamFile, SIGNAL(clicked()), this, SLOT(openParamFile()));
//        
//          vbox2->addWidget(butCalibration);
//          vbox2->addWidget(butOpenParamFile);
//        
//        
//          QGroupBox * gridMat = new QGridLayout(this);
//            QLabel * mat00 = new QLabel(this);  QLabel * mat01 = new QLabel(this); QLabel * mat02 = new QLabel(this);
//            QLabel * mat10 = new QLabel(this);  QLabel * mat11 = new QLabel(this); QLabel * mat12 = new QLabel(this);
//            QLabel * mat20 = new QLabel(this);  QLabel * mat21 = new QLabel(this); QLabel * mat22 = new QLabel(this);
//          
//        hbox1->addLayout(vbox2);
//        hbox1->addLayout(vbox2);
//        
      
      
      vbox1->addWidget(groupTop);
      vbox1->addWidget(labBot);
      
      
      
    pageCal->setLayout(vbox1);

    pageScan = new QWidget;
    
    
  tabs->addTab(pageCal, "Calibration");
  tabs->addTab(pageScan, "Scan");
  
  tabs->addAction(actionOpenParamFile);
  tabs->addAction(actionSaveFile);
  tabs->addAction(actionQuit);
  
  loadDefaultParameters();
 
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


void MainWin::openParamFile()
{
//  std::string     param_file                           = "param/cal_param.xml";
//  std::string     cal_needed                           = "yes";
//  double         cal_circular_pattern_radius          = 5.6;
//  int            cal_circular_pattern_number_points   = 20;
//  std::string     param_name_matrix                    = "matM";
//  std::string     param_name_normal                    = "vecN";
//  std::string     scan_output_file                     = "data/scan_output.xyz";
//  double         scan_step_angle                      = 1;

  QString filename = QFileDialog::getOpenFileName( 
                                                    this, 
                                                    tr("Open"), 
                                                    QDir::currentPath(), 
                                                    tr("Parameter files (*.xml)")
                                                 );
                                                 
                                                 
  std::string lM = "matM";
  std::string lN = "vecN";
  
//  if(pNameMatrix.size()==0 || pNameNormal.size()==0 || pNameMatrix.find(" ") != std::string::npos || pNameNormal.find(" ") != std::string::npos)
//  {
//    std::cerr << "Warning: bad name for data variables. The transformation matrix will be saved in the XML file as \"matM\" and the normal vector as \"vecN\"." << std::endl;
//    lM = "matM";
//    lN = "vecN";
//  }
//  paramFile = "cal_param.xml";
  cv::FileStorage fs(filename.toUtf8().constData(), cv::FileStorage::READ); // parameters file
  
//  cv::Mat matM,vecN;
  matM.create(3,4,cv::DataType<double>::type);
  vecN.create(4,1,cv::DataType<double>::type);
  
  fs[lM] >> matM;
  fs[lN] >> vecN;
  
  // upate display of matrix
  
  for(int i=0;i<3;++i)
  {
    for(int j=0;j<4;++j)
    {
      labMat[i][j]->setText(QString::number(matM.at<double>(i,j)));
    }
  }
  
  for(int i=0;i<3;++i)
  {
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
}

void MainWin::calibrate()
{
  Calibration cal(param_file);
  cal.circularPattern(cal_circular_pattern_number_points,cal_circular_pattern_radius);
  cal.save("matM","vecN");

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


void MainWin::newScan()
{
  


}







