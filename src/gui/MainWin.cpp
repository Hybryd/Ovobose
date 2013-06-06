#include "MainWin.h"
 
MainWin::MainWin()
{
  QMenu *menuFile = menuBar()->addMenu("&File");
  
  // Open
//  QMenu *menuOpenFile = menuBar()->addMenu("&Open");
  QAction *actionOpenFile = new QAction("&Open", this);
  menuFile->addAction(actionOpenFile);
  connect(actionOpenFile, SIGNAL(triggered()), this, SLOT(openFile()));
  actionOpenFile->setShortcut(QKeySequence("Ctrl+O"));

  // Save
//  QMenu *menuSaveFile = menuBar()->addMenu("&Save");
  QAction *actionSaveFile = new QAction("&Save", this);
  menuFile->addAction(actionSaveFile);
  connect(actionSaveFile, SIGNAL(triggered()), this, SLOT(saveFile()));
  actionSaveFile->setShortcut(QKeySequence("Ctrl+S"));
    
  // Quit
  QAction *actionQuitter = new QAction("&Quitter", this);
  menuFile->addAction(actionQuitter);
  connect(actionQuitter, SIGNAL(triggered()), qApp, SLOT(quit()));
  actionQuitter->setShortcut(QKeySequence("Ctrl+Q"));  
  
  
  // Central widget
  QWidget *central = new QWidget;
  setCentralWidget(central);
  QFormLayout *layout = new QFormLayout;




  central->setLayout(layout);
  setCentralWidget(central);
 
}


void MainWin::openFile()
{
  QString filename = QFileDialog::getOpenFileName( 
                                                    this, 
                                                    tr("Open"), 
                                                    QDir::currentPath(), 
                                                    tr("Parameter files (*.txt);;All files (*.*)")
                                                 );
//  if( !filename.isNull() )
//  {
//    qDebug( filename.toAscii() );
//  }
}


void MainWin::saveFile()
{
  QString filename = QFileDialog::getSaveFileName(  
                                                    this, 
                                                    tr("Save"), 
                                                    QDir::currentPath(), 
                                                    tr("All files (*.*)")
                                                 );
//  if( !filename.isNull() )
//  {
//    qDebug( filename.toAscii() );
//  }
}


void MainWin::newScan()
{



}




