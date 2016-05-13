#include "kinectinterface.h"
#include <qDebug>
#include <QWidget>
#define COLORSCALE 0.4

KinectInterface::KinectInterface(QObject *parent) : QObject(parent)
{
    kinect = new MyKinect();
}

void KinectInterface::startCapture()
{
    kinect->initialize_kinect();


    nframes = 0; // init
    frameRate = 30;
    timerId = startTimer(1000 / frameRate); // in msec
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

    qDebug() << timerId;
    if (event->timerId() == timerId)
    {


        cv::Mat colorMat = kinect->get_colorframe(COLORSCALE);
        cv::imshow( "Color", colorMat );



        //IplImage *frame=webcam->getImage();



/*

        QImage image = IplImage2QImage(frame); // convert
        pixmap = QPixmap::fromImage(image); // convert
        //repaint(); // immediate repaint

        emit getImageFromCamera(pixmap);
*/
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
QImage KinectInterface::IplImage2QImage(const IplImage *iplImage)
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
