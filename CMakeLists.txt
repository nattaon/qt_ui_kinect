cmake_minimum_required(VERSION 2.8)
project( MyProjectBpp )

if(CMAKE_CONFIGURATION_TYPES)
    message("Multi-configuration generator")
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" 
	CACHE STRING "My multi config types" FORCE)
else()
    message("Single-configuration generator")
endif()


SET(Kinect_DIR "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409")
include_directories(${Kinect_DIR}/inc)

SET(OpenCV_DIR "C:/opencv249/build")
find_package( OpenCV REQUIRED )
find_package (VTK REQUIRED) #for QT5_WRAP_CPP command 



file(GLOB project_SOURCES *.cpp)
file(GLOB project_HEADERS *.h)
file(GLOB project_FORMS *.ui)

#set  (project_SOURCES main.cpp mainwindow.cpp camerainterface.cpp webcam.cpp)
#set  (project_HEADERS mainwindow.h camerainterface.h webcam.h)
#set  (project_FORMS   mainwindow.ui)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (MyProjectBpp 
${project_SOURCES} 
${project_FORMS_HEADERS} 
${project_HEADERS_MOC})

#set (kinect_lib_file )
target_link_libraries( MyProjectBpp ${OpenCV_LIBS} ${Kinect_DIR}/Lib/x64/Kinect20.lib)
qt5_use_modules (MyProjectBpp Widgets)
