#include "camerainterface.h"
#include <qDebug>
#include <QWidget>

CameraInterface::CameraInterface(QObject *parent) : QObject(parent)
{
    webcam = new WebCam();
}

void CameraInterface::startCapture()
{

    webcam->openCamera();
	nframes = 0; // init
    frameRate = 30;
    timerId = startTimer(1000 / frameRate); // in msec
    time.start(); // start time

}
void CameraInterface::stopCapture()
{
	qDebug() << "stopCapture";


	killTimer(timerId);


	pixmap.fill(Qt::black);
	emit getImageFromCamera(pixmap);

	webcam->closeCamera();
}

void CameraInterface::timerEvent(QTimerEvent *event)
{
    
	//qDebug() << timerId;
    if (event->timerId() == timerId)
    {
        IplImage *frame=webcam->getImage();

        QImage image = IplImage2QImage(frame); // convert
        pixmap = QPixmap::fromImage(image); // convert
        //repaint(); // immediate repaint

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


// Convert OpenCV's IplImage to QImage.
QImage CameraInterface::IplImage2QImage(const IplImage *iplImage)
{
    if (!iplImage) return QImage();

    int height = iplImage->height;
    int width = iplImage->width;
    if(iplImage->depth == IPL_DEPTH_8U && iplImage->nChannels == 3) // colour image
    {
        const uchar *qImageBuffer = (const uchar*) iplImage->imageData;
        QImage img(qImageBuffer, width, height,
        QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else if(iplImage->depth == IPL_DEPTH_8U && iplImage->nChannels == 1) // gray image
    {
        const uchar *qImageBuffer = (const uchar*) iplImage->imageData;
        QImage img(qImageBuffer, width, height,
        QImage::Format_Indexed8);
        QVector<QRgb> colorTable; // set up colour table
        for (int i = 0; i < 256; i++)
            colorTable.append(qRgb(i, i, i));
        img.setColorTable(colorTable);
        return img;
    }
    else
    {
        qWarning() << "Image cannot be converted.";
        return QImage();
    }
}
