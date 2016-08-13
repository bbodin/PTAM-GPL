cmake_minimum_required(VERSION 2.6.0)

SET(appname ptam)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3                  -w ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3  --std=c++14 -w ")

find_package(LAPACK REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(CVD REQUIRED)
include_directories(${CVD_INCLUDE_DIR})


find_package(GVARS REQUIRED)
include_directories(${GVARS_INCLUDE_DIR})

find_package(OpenGL   REQUIRED)

find_package(TooN   REQUIRED)
include_directories(${TOON_INCLUDE_PATHS})

include_directories(./Src)

file(GLOB srcs Src/*.cc Src/*.h)

set(atan_srcs 	./Src/ATANCamera.h
				./Src/ATANCamera.cc
				)
				
set(calib_srcs 	./Src/CalibCornerPatch.h
				./Src/CalibImage.h
				./Src/CameraCalibrator.cc
				./Src/CameraCalibrator.h
				./Src/CalibCornerPatch.cc
				./Src/CalibImage.cc
				)
				
set(base_srcs 	./Src/ShiTomasi.cc
				./Src/VideoSource.h
				./Src/PatchFinder.h
				./Src/MEstimator.h
				./Src/HomographyInit.cc
				./Src/MapMaker.cc
				./Src/ARDriver.cc
				./Src/ARDriver.h
				./Src/MapMaker.h
				./Src/LevelHelpers.h
				./Src/TrackerData.h
				./Src/MapPoint.cc
				./Src/MapPoint.h
				./Src/KeyFrame.h
				./Src/MapViewer.h
				./Src/MapViewer.cc
				./Src/Relocaliser.cc
				./Src/Map.cc
				./Src/SmallMatrixOpts.h
				./Src/MiniPatch.cc
				./Src/SmallBlurryImage.cc
				./Src/Tracker.h
				./Src/Bundle.cc
				./Src/HomographyInit.h
				./Src/EyeGame.h
				./Src/PatchFinder.cc
				./Src/KeyFrame.cc
				./Src/EyeGame.cc
				./Src/ShiTomasi.h
				./Src/Tracker.cc
				./Src/Relocaliser.h
				./Src/Map.h
				./Src/SmallBlurryImage.h
				./Src/MiniPatch.h
				./Src/Bundle.h 
				)

set(gui_srcs 	./Src/GLWindow2.h
				./Src/GLWindow2.cc
				./Src/GLWindowMenu.h
				./Src/GLWindowMenu.cc
				./Src/OpenGL.h
				./Src/System.h
				./Src/System.cc
				)
set(video_source_srcs 	
				./Src/VideoSource_Linux_CVD.cc
				)
				
add_library(${appname}-original  
            ${base_srcs} ${atan_srcs}
)


target_link_libraries(${appname}-original
                      ${GVARS_LIBRARY}
                      ${CVD_LIBRARY}
                      lapack
)
 
add_slambench_binary(${appname}   gui   original "${gui_srcs};${video_source_srcs}" "" "${OPENGL_LIBRARIES};${CVD_LIBRARY};pthread")

