#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

#include "FindBody.h"


using namespace cv;
using namespace std;


void FindBody::onKey(char key)
{
  
};


void FindBody::onFrame(Mat& frame)
{
  

 if(!isBackgroundInitialized){
    cvtColor(frame, avg, COLOR_BGR2GRAY);
    avg.convertTo(avg, CV_32FC1);
    isBackgroundInitialized = true;
    return;
} 
  
  
  cvtColor(frame, gray, COLOR_BGR2GRAY);
  GaussianBlur(gray, gray, cv::Size(21, 21), 0);
  
  accumulateWeighted(gray, avg, 0.5);
  Mat m;
  convertScaleAbs(avg, m);
  absdiff(gray, m, frameDelta);
  
  Mat thresh;
  threshold(frameDelta,thresh, 5, 255,THRESH_BINARY);
  dilate(thresh, thresh, 0, Point(-1, -1), 2);
  vector<vector<Point> > contours;
  findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  
  string text;
  
  for (uint i = 0; i < contours.size(); i++) {
    vector<Point> c = contours[i];
    if(contourArea(c) < 9000) {
      continue;
      
    } else {
      Rect face_i = boundingRect(c);
      rectangle(frame, face_i, CV_RGB(0, 255,0), 1);
      printCommand(frame, face_i);
      text = "Occupied";
    }
  }
  
  imshow("find_body", frame);
}


  void FindBody::printCommand(Mat& frame, Rect& faceCenter)
  {
      std::cout << "{\"mode\": " << "\"find_body\"" << ", "<<"\"x\": " << faceCenter.x << ", " << "\"y\": "  << faceCenter.y << ", "<< "\"width\": "  << faceCenter.width << ", " << "\"height\": " << faceCenter.height << " }" << std::endl;;
  }




