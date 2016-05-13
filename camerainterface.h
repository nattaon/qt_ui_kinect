#ifndef CAMERAINTERFACE_H
#define CAMERAINTERFACE_H
#include <QtCore>
#include <QObject>
#include <QWidget>
#include <QTime>
#include "webcam.h"

class CameraInterface : public QObject
{
	Q_OBJECT
public:
    CameraInterface(QObject *parent=0);
    void startCapture();
    void stopCapture();

signals:
    void getImageFromCamera(QPixmap pixmap);


protected:
    void timerEvent(QTimerEvent *event);

private:
    WebCam *webcam;

    int timerId;
    int frameRate; // input frame rate
    QPixmap pixmap;
    int nframes; // used to calculate actual frame rate
    QTime time; // used to calculate actual frame rate

    QImage IplImage2QImage(const IplImage *iplImage);

};

#endif // CAMERAINTERFACE_H
