#include "kinectinterface.h"
#include <qDebug>
#include <QWidget>
#define COLORSCALE 0.3

KinectInterface::KinectInterface(QObject *parent) : QObject(parent)
{
    kinect = new MyKinect();
}

void KinectInterface::startCapture()
{
    kinect->initialize_kinect();


    nframes = 0; // init
    frameRate = 30;

    //timerId = startTimer(1000 / frameRate); // in msec
    timerId = startTimer(100); // in msec

    time.start(); // start time

}
void KinectInterface::stopCapture()
{
    qDebug() << "stopCapture";


    killTimer(timerId);


    pixmap.fill(Qt::black);
    emit getImageFromCamera(pixmap);

    kinect->release_kinect();
}

void KinectInterface::timerEvent(QTimerEvent *event)
{

    //qDebug() << timerId;
    if (event->timerId() == timerId)
    {


        cv::Mat colorMat = kinect->get_colorframe(COLORSCALE);
		//cv::imshow( "Color", colorMat );


		cv::cvtColor(colorMat, colorMat, CV_BGR2RGB);
		QImage image = QImage((uchar*)colorMat.data, colorMat.cols, colorMat.rows, colorMat.step, QImage::Format_RGB888);
		//QImage image = Mat2QImage(colorMat); // convert
		pixmap = QPixmap::fromImage(image); // convert


		emit getImageFromCamera(pixmap);
		

        if (++nframes == 50)
        {
            qDebug("frame rate: %f", // actual frame rate
            (float) nframes * 1000 / time.elapsed());
            nframes = 0;
            time.restart();
        }


    }
    else
        QObject::timerEvent(event);

}


// Convert OpenCV's Mat to QImage.
QImage KinectInterface::Mat2QImage(const cv::Mat3b &src)
{
	//strip appear

	QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
	for (int y = 0; y < src.rows; ++y) {
		const cv::Vec3b *srcrow = src[y];
		QRgb *destrow = (QRgb*)dest.scanLine(y);
		for (int x = 0; x < src.cols; ++x) {
			destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
		}
	}
	
	return dest;
}