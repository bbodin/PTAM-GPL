/*
* Autor : Arnaud GROSJEAN (VIDE SARL)
* This implementation of VideoSource allows to use OpenCV as a source for the video input
* I did so because libCVD failed getting my V4L2 device
*
* INSTALLATION :
* - Copy the VideoSource_Linux_OpenCV.cc file in your PTAM directory
* - In the Makefile:
*	- set the linkflags to
	LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lblas -llapack -lGVars3 -lcvd -lcv -lcxcore -lhighgui
*	- set the videosource to 
	VIDEOSOURCE = VideoSource_Linux_OpenCV.o
* - Compile the project
* - Enjoy !
* 
* Notice this code define two constants for the image width and height (OPENCV_VIDEO_W and OPENCV_VIDEO_H)
*/

#include "VideoSource.h"
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>

#include <unistd.h>
#include <cvd/image_io.h>

using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace cv;

#define OPENCV_VIDEO_W 640
#define OPENCV_VIDEO_H 480

VideoSource::VideoSource()
{ 
mirSize = ImageRef(OPENCV_VIDEO_W, OPENCV_VIDEO_H);
};

ImageRef VideoSource::Size()
{ 
  return mirSize;
};

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
this->incrementCounter();
int counter = this->getCounter();
std::string filename;
// printf("The value of the counter is %d \n", counter);
  if(counter < 10){
   filename =  "0000000" + to_string(counter);
  }
  else if(counter < 100){
    filename =  "000000" + to_string(counter);
  }
  else if(counter < 1000){
    filename =  "00000" + to_string(counter);
  }
  else{
    filename = "0000" + to_string(counter);
  }
  filename = "/home/toky/work/pamela/slambench2/datasets/ptam/" + filename + ".png";
  imBW = CVD::img_load(filename);
  imRGB = CVD::img_load(filename);
}


