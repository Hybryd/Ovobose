
#ifndef QOPENCVWIDGET_H
#define QOPENCVWIDGET_H

#include <cv.h>
#include <highgui.h>
#include <QPixmap>
#include <QLabel>
#include <QWidget>
#include <QVBoxLayout>
#include <QImage>

class QOpenCVWidget : public QWidget
{
  Q_OBJECT

signals:
  void mouseMove( QMouseEvent* e );

protected:
  QLabel *imagelabel;
  QVBoxLayout *layout;

  QImage image;

  void mouseMoveEvent( QMouseEvent* e );

public:
  QOpenCVWidget(QWidget *parent = 0);
  ~QOpenCVWidget(void);
  void putImage(cv::Mat &);
  void putTextOnly(QString s);


};

#endif
