#ifndef MYKINECT_H
#define MYKINECT_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Windows.h>
#include <Kinect.h>
#include <vector>

using namespace cv;

class MyKinect
{
public:
    MyKinect();
	~MyKinect();
    void initialize_kinect();
    void release_kinect();
    void openKinect();
    IColorFrameReader* get_colorframereader();
    IDepthFrameReader* get_depthframereader();
    ICoordinateMapper* get_coordinatemapper();
	Mat get_colorframe(float COLORSCALE);
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL){
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

private:
    HRESULT hResult;

    int colorWidth;
    int colorHeight;
    int depthWidth;
    int depthHeight;

    unsigned int colorBufferSize;
    unsigned int depthBufferSize;

    std::vector<RGBQUAD> colorBuffer;
    std::vector<UINT16> depthBuffer;


    IKinectSensor* pSensor;
    IColorFrameSource* pColorSource;
    IFrameDescription* pColorDescription;
    IDepthFrameSource* pDepthSource;
    IFrameDescription* pDepthDescription;

    IColorFrameReader* pColorReader;
    IDepthFrameReader* pDepthReader;
    ICoordinateMapper* pCoordinateMapper;

};

#endif // MYKINECT_H
