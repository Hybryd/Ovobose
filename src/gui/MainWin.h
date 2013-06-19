#ifndef MAINWIN_H
#define MAINWIN_H


#define BORDER 5
#define HEIGHT_PROGRESS 15


#include <cv.h> 
#include <iostream>
#include <QtGui>
#include "../Calibration.h"
#include "../DataConverter.h"
#include "../Scan.h"

 
class MainWin : public QMainWindow
{
  Q_OBJECT


protected:
  cv::Mat         matM;
  cv::Mat         vecN;
  std::string     param_file;
//  bool           cal_needed;
  double         cal_circular_pattern_radius;
  int            cal_circular_pattern_number_points;
//  std::string     param_name_matrix;
//  std::string     param_name_normal;
  std::string     scan_output_file;
  double         scan_step_angle;
  
  
  // Qt variables
  QAction                                 * actionOpenParamFile;
  QAction                                 * actionSaveFile;
  QAction                                 * actionQuit;
  QTabWidget                              * tabs;
  QWidget                                 * pageCal;
  QVBoxLayout                             * vbox1;
  QGroupBox                               * groupTop;
  QHBoxLayout                             * hbox1;
  QGroupBox                               * groupBut;
  QVBoxLayout                             * vbox2;
  QPushButton                             * butCalibration;
  QPushButton                             * butOpenParamFile;
  QGroupBox                               * groupMat;
  QGridLayout                             * gridMat;
  std::vector< std::vector< QLabel * > >    labMat;
  QGroupBox                               * groupVec;
  QGridLayout                             * gridVec;
  std::vector< QLabel * >                   labVec;
  QLabel                                  * labBot;
  QWidget                                 * pageScan;
  QProgressBar                            * progress;

public:
  MainWin();
  void loadDefaultParameters();
  bool checkIfCamera();
    
public slots :
  
  void openParamFile();
  bool checkIfParamOk();
  void calibrate();
  void saveFile();
  void newScan();

};
 
#endif
