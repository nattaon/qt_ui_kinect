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


void MyKinect::get_depthframe()
{
    // Acquire Latest Depth Frame
    IDepthFrame* pDepthFrame = nullptr;
    hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
    if( SUCCEEDED( hResult ) ){
        // Retrieved Depth Data
        hResult = pDepthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] );
        if( FAILED( hResult ) ){
            std::cerr << "Error : IDepthFrame::CopyFrameDataToArray()" << std::endl;
        }
    }
    SafeRelease( pDepthFrame );
}


void MyKinect::mapping_pointcloud(pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud)
{
    //pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud( new pcl::PointCloud<PointTypeXYZRGB>() );
    pointcloud->width = static_cast<uint32_t>( depthWidth );
    pointcloud->height = static_cast<uint32_t>( depthHeight );
    pointcloud->is_dense = false;

    pointcloud->clear();

        for( int y = 0; y < depthHeight; y++ ){
            for( int x = 0; x < depthWidth; x++ ){
                pcl::PointXYZRGB point;

                DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
                UINT16 depth = depthBuffer[y * depthWidth + x];

                // Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
                ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
                pCoordinateMapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );
                int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );
                int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );
                if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                    RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
                    point.b = color.rgbBlue;
                    point.g = color.rgbGreen;
                    point.r = color.rgbRed;
                }

                // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
                CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
                pCoordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
                if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                    point.x = cameraSpacePoint.X;
                    point.y = cameraSpacePoint.Y;
                    point.z = cameraSpacePoint.Z;
                }

                pointcloud->push_back( point );
            }
        }


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
