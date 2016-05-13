#ifndef WEBCAM_H
#define WEBCAM_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class WebCam
{
public:
    WebCam();
    void openCamera();
    IplImage* getImage();
    void closeCamera();

private:
    CvCapture* capture;

};

#endif // WEBCAM_H
