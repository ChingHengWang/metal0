#include <iostream>

#include "ros/ros.h"

#include "FaceDetector.h"


using std::cout;
using std::endl;
using namespace cv;


const char* FaceDetector::HAAR_FRONTALFACE_DEFAULT_PATH = "/home/odroid/catkin_ws/src/visionint/res/haarcascades/haarcascade_frontalface_default.xml";
const char* FaceDetector::HAAR_FRONTALFACE_ALT_PATH = "/home/odroid/catkin_ws/src/visionint/res/haarcascades/haarcascade_frontalface_alt.xml";

void FaceDetector::onFrame(Mat& frame)
{
    Processor::onFrame(frame);
}


void FaceDetector::onKey(char key)
{
    Processor::onKey(key);
}

vector< Rect_<int> > FaceDetector::detectFaces(Mat& gray)
{
  vector< Rect_<int> > faces;
  int minNeighbours = 3;
  double scaleFactor = 1.1;
  Size minSize = Size(MIN_FACE_WIDTH, MIN_FACE_HEIGHT);
  haarCascade.detectMultiScale(gray, faces, scaleFactor, minNeighbours, CV_HAAR_DO_CANNY_PRUNING, minSize);
  //std::cout << "found " << faces.size() << " faces" << std::endl;
  
  return faces;
}



void FaceDetector::loadHaarCascades(const char* filename)
{
  cout << "loading haar from " << filename << endl;
  //cwd is src
  
  std::string fn ("");
  fn += filename;  
  haarCascade.load(fn);
  if(haarCascade.empty()){
    ROS_ERROR("couldn't load haar from %s", fn.c_str());
    cout << "couldn't load haar from " << fn << endl;
  }
}

FaceDetector::FaceDetector()
{
  loadHaarCascades(HAAR_FRONTALFACE_DEFAULT_PATH);
}
