#ifndef MAINWIN_H
#define MAINWIN_H
 
#include <iostream>
#include <QtGui>
#include "../Scan.h"

 
class MainWin : public QMainWindow
{
  Q_OBJECT


protected:


public:
  MainWin();
    
public slots :
  void openFile();
  void saveFile();
  void newScan();

};
 
#endif
