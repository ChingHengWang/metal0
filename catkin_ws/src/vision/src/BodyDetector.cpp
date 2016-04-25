#include <iostream>
#include "math.h"


#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Eigen>
#include "opencv2/core/eigen.hpp"


#include "BodyDetector.h"


using std::cout;
using std::endl;
using namespace cv;


const char* BodyDetector::HAAR_FULLBODY_PATH = "/home/odroid/catkin_ws/src/vision/res/haarcascades/haarcascade_mcs_upperbody.xml";

void BodyDetector::onFrame(Mat& frame)
{
    Processor::onFrame(frame);
}


void BodyDetector::onKey(char key)
{
    Processor::onKey(key);
}

vector< Rect_<int> > BodyDetector::detectBodies(Mat& frame)
{
  Mat gray;
  cv::cvtColor(frame, gray, CV_BGR2GRAY);
  
  vector< Rect_<int> > bodies;
  int minNeighbours = 2;
  double scaleFactor = 1.1;
  Size minSize = Size(MIN_BODY_WIDTH, MIN_BODY_HEIGHT);
  haarCascade.detectMultiScale(gray, bodies, scaleFactor, minNeighbours, CV_HAAR_DO_CANNY_PRUNING, minSize);
  if(bodies.size() > 0) {
    std::cout << "found " << bodies.size() << " bodies" << std::endl;
  }
  
  for(uint i = 0; i < bodies.size(); i++)
  {
    //std::cout << "body: " << bodies[i] << std::endl;
    cv::rectangle(frame, bodies[i], CV_RGB(0, 255,0), 1);
  }
  
  return bodies;
}


void BodyDetector::calculateCovar(Mat& input, Mat& result)
{
  int flags = cv::COVAR_COLS | cv::COVAR_NORMAL;
  int ddepth = CV_32F;
  Mat mean;
  
  //cout << "input" << endl << format(input, "python") << endl;
  calcCovarMatrix(input, result, mean, flags, ddepth);
  
  result = result/(input.cols - 1);
  //cout << "result" << endl << format(result, "python") << endl;
}


cv::Mat BodyDetector::getDescriptor(Mat& roi)
{
  cv::Size original = roi.size();
  float r = float(BODY_HEIGHT) / original.height;
  cv::Size scaleTo = cv::Size(int(original.width * r), BODY_HEIGHT);
  
  cv::resize(roi, roi, scaleTo, CV_INTER_AREA);
  
  
  Mat gxx, gxy, gyy, gray;
  cvtColor(roi, gray, CV_BGR2GRAY);
  
  int ddepth = CV_32F;
  int dx = 0;
  int dy = 0;
  int ksize = 3;
  cv::Sobel(gray, gxx, ddepth, 2, 0, ksize);
  cv::Sobel(gray, gxy, ddepth, 1, 1, ksize);
  cv::Sobel(gray, gyy, ddepth, 0, 2, ksize);
  
  Mat pixels;
  gray.convertTo(pixels, CV_32F);
  
  
  Mat fgxx = gxx.reshape(0,1);
  Mat fgxy = gxy.reshape(0,1);
  Mat fgyy = gyy.reshape(0,1);
  Mat fpixels = pixels.reshape(0,1);
  
  
  cout << "fgxy" << format(fgxy, "python") << endl;
  
  
  Mat input(4, fgxx.cols, DataType<float>::type); 
  
  fgxx.row(0).copyTo(input.row(0));
  fgxy.row(0).copyTo(input.row(1));
  fgyy.row(0).copyTo(input.row(2));
  fpixels.row(0).copyTo(input.row(3));
  
  Mat result;
  calculateCovar(input, result);
  return result;
}



void BodyDetector::loadHaarCascades(const char* filename)
{
  cout << "loading haar from " << filename << endl;
  //cwd is src
  
  std::string fn ("");
  fn += filename;  
  haarCascade.load(fn);
  if(haarCascade.empty()){
    cout << "couldn't load haar from " << fn << endl;
  }
}

BodyDetector::BodyDetector()
{
  loadHaarCascades(HAAR_FULLBODY_PATH);
}
