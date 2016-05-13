#include "mykinect.h"
#include <iostream>
#include <sstream>


MyKinect::MyKinect()
{
    hResult = S_OK;

    pSensor = nullptr;

    colorWidth = 0;
    colorHeight = 0;
    depthWidth = 0;
    depthHeight = 0;

}
MyKinect::~MyKinect()
{

	pSensor->Close();
	pSensor->Release();
}
void MyKinect::initialize_kinect()
{

    openKinect();

}

void MyKinect::openKinect()
{
	/*
	ERROR_CHECK(::GetDefaultKinectSensor(&pSensor));
	ERROR_CHECK(pSensor->Open());
	BOOLEAN isOpen = false;
	ERROR_CHECK(pSensor->get_IsOpen(&isOpen));
	std::cout << "Kinect is " << (isOpen ? "Open" : "Not Open") << std::endl;
*/

	// Create Sensor Instance	
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		//return -1;
	}

	// Open Sensor
	hResult = pSensor->Open();
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		//return -1;
	}

	pColorReader = get_colorframereader();
	pDepthReader = get_depthframereader();
	pCoordinateMapper = get_coordinatemapper();

	colorBufferSize = colorWidth * colorHeight * 4 * sizeof(unsigned char);
	depthBufferSize = depthWidth * depthHeight * sizeof(unsigned short);

	// To Reserve Color Frame Buffer
	//std::vector<RGBQUAD> colorBuffer( colorWidth * colorHeight );
	colorBuffer.resize(colorWidth * colorHeight);

	// To Reserve Depth Frame Buffer
	//std::vector<UINT16> depthBuffer( depthWidth * depthHeight );
	depthBuffer.resize(depthWidth * depthHeight);

}

IColorFrameReader* MyKinect::get_colorframereader()
{
	// Retrieved Color Frame Source	
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		//return -1;
	}
	// Open Color Frame Reader
	//IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		//return -1;
	}
	// Retrieved Color Frame Size

	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		//return -1;
	}
	pColorDescription->get_Width(&colorWidth); // 1920
	pColorDescription->get_Height(&colorHeight); // 1080

	return pColorReader;
}
IDepthFrameReader* MyKinect::get_depthframereader()
{
	//// DO get depth image	

	// Retrieved Depth Frame Source	
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		//return -1;
	}
	// Open Depth Frame Reader
	//IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		//return -1;
	}
	// Retrieved Depth Frame Size

	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		//return -1;
	}
	pDepthDescription->get_Width(&depthWidth); // 512
	pDepthDescription->get_Height(&depthHeight); // 424	

	return pDepthReader;
}
ICoordinateMapper* MyKinect::get_coordinatemapper()
{
	//// Retrieved Coordinate Mapper
	//ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		//return -1;
	}

	return pCoordinateMapper;
}


Mat MyKinect::get_colorframe(float COLORSCALE)
{
	cv::Mat colorBufferMat(colorHeight, colorWidth, CV_8UC4);
	cv::Mat colorMat(colorHeight * COLORSCALE, colorWidth * COLORSCALE, CV_8UC4);

	// Acquire Latest Color Frame
	IColorFrame* pColorFrame = nullptr;
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult)){

		// Retrieved Color Data for show in window
		hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize,
			reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);

		if (SUCCEEDED(hResult)){

			cv::resize(colorBufferMat, colorMat, cv::Size(), COLORSCALE, COLORSCALE);
			cv::flip(colorMat, colorMat, 1);

			// Retrieved Color Data for pcl
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD),
				reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
			if (FAILED(hResult)){
				std::cerr << "Error : IColorFrame::CopyConvertedFrameDataToArray()" << std::endl;
			}
		}
	}
	SafeRelease(pColorFrame);

	return colorMat;
}

void MyKinect::release_kinect()
{
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);
	SafeRelease(pCoordinateMapper);

	SafeRelease(pColorSource);
	SafeRelease(pDepthSource);
	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);
	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
}