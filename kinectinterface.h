#ifndef KINECTINTERFACE_H
#define KINECTINTERFACE_H
#include <QtCore>
#include <QObject>
#include <QWidget>
#include <QTime>
#include "mykinect.h"

class KinectInterface : public QObject
{
    Q_OBJECT
public:
    KinectInterface(QObject *parent=0);
    void startCapture();
    void stopCapture();

signals:
    void getImageFromCamera(QPixmap pixmap);


protected:
    void timerEvent(QTimerEvent *event);

private:
    MyKinect *kinect;


    int timerId;
    int frameRate; // input frame rate
    QPixmap pixmap;
    int nframes; // used to calculate actual frame rate
    QTime time; // used to calculate actual frame rate

	QImage Mat2QImage(const cv::Mat3b &src);



};

#endif // KINECTINTERFACE_H
