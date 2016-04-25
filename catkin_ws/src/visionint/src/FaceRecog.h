#ifndef FACERECOGNIZER_H_
#define FACERECOGNIZER_H_

#include "FaceDetector.h"
#include "src/FeedbackPublisher.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>

using cv::Mat;


class FaceRecog : public FaceDetector
{
  
private:
  static const char* WINDOW_NAME;


//face tracking
  cv::Rect selection;
  int vmin = 10, vmax = 256, smin = 30;
  Mat  hsv, hue, mask, hist, histimg, backproj;

  bool tracking = false;
  
  cv::Rect trackWindow;
  int hsize = 16;
  float hranges[2] = {0,180};
  const float* phranges = hranges;
  
  void startDetection(Mat frame);
  void displayFoundFace(Mat& face);
  void initFaceRecognizer();
  void increaseRecognizerThreshold(double inc);
  void initTracking(cv::Rect face, Mat& f);
  void track(Mat& frame);
  void showRoi(Mat& frame);
  void drawArrow(Mat& frame, cv::Point to);
  void printCommand(Mat& frame, cv::Point faceCenter);
  void predict(Mat& original);
  void findFace(Mat& frame, Mat& result);

  FeedbackPublisher* feedbackPublisher;
  
  cv::Ptr<cv::FaceRecognizer> faceRecognizer;
  
public:
    FaceRecog(FeedbackPublisher* feedbackPublisher);
    void loadFaces();
    virtual void onFrame(Mat& frame);
    virtual void onKey(char key);
};

#endif /* FACERECOGNIZER_H_ */
