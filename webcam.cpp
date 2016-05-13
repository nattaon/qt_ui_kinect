#include "webcam.h"
#include <qDebug>
#include <QWidget>


WebCam::WebCam()
{

}


void WebCam::openCamera()
{
    capture = cvCaptureFromCAM(0);

    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
}


IplImage* WebCam::getImage()
{
    IplImage *frame = cvQueryFrame(capture);
    return frame;
}


void WebCam::closeCamera()
{
    cvReleaseCapture(&capture);
}


