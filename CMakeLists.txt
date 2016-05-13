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

SET(OpenCV_DIR "C:/opencv249/build")
find_package( OpenCV REQUIRED )

find_package (PCL 1.7.2 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS} ${Kinect_DIR}/inc)
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})


find_package (VTK REQUIRED) #for QT5_WRAP_CPP command 
include(${VTK_USE_FILE})

file(GLOB project_SOURCES *.cpp)
file(GLOB project_HEADERS *.h)
file(GLOB project_FORMS *.ui)
set (VTK_LIBRARIES)


QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (MyProjectBpp ${project_SOURCES} 
							  ${project_FORMS_HEADERS} 
							  ${project_HEADERS_MOC})

#set (kinect_lib_file )
target_link_libraries( MyProjectBpp ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${Kinect_DIR}/Lib/x64/Kinect20.lib)

qt5_use_modules (MyProjectBpp Widgets)
