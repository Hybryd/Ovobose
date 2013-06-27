#include "QOpenCVWidget.h"
#include <stdio.h>

QOpenCVWidget::QOpenCVWidget(QWidget *parent) : QWidget(parent)
{
  layout = new QVBoxLayout;
  imagelabel = new QLabel;
  QImage qimagetmp(314,314,QImage::Format_RGB32); // why not
  image = qimagetmp;
  layout->addWidget(imagelabel);
  for (int x = 0; x < 314; x ++)
  {
    for (int y =0; y < 314; y++)
    {
      image.setPixel(x,y,qRgb(x, y, y));
    }
  }
  imagelabel->setPixmap(QPixmap::fromImage(image));
  setLayout(layout);
}

QOpenCVWidget::~QOpenCVWidget(void)
{
}

void QOpenCVWidget::putImage(cv::Mat & cvimage)
{
  if ( (cvimage.cols != image.width()) || (cvimage.rows != image.height()) )
  {
    QImage temp(cvimage.cols, cvimage.rows, QImage::Format_RGB32);
    image = temp;
  }
  for (int x = 0; x < cvimage.cols; x ++)
  {
    for (int y =0; y < cvimage.rows; y++)
    {
      image.setPixel(x,y,qRgb(cvimage.at<cv::Vec3b>(y,x)[2], cvimage.at<cv::Vec3b>(y,x)[1], cvimage.at<cv::Vec3b>(y,x)[0]));
    }
  }  
  imagelabel->setPixmap(QPixmap::fromImage(image));
}


