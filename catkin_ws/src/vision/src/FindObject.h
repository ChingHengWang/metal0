#ifndef FIND_OBJECT_H_
#define FIND_OBJECT_H_

#include "processor.h"

class FindObject : public Processor {
  bool selectObject = false;
  bool objectSelected = false;
  bool objectLearned = false;
  Point origin;
  Rect selection;
  
std::vector<cv::KeyPoint> objectKeypoints;
std::vector<cv::KeyPoint> sceneKeypoints;
cv::Mat objectDescriptors;
cv::Mat sceneDescriptors;
cv::Ptr<cv::FeatureDetector> detector;
cv::Ptr<cv::DescriptorExtractor> extractor;
Mat objectImg;
public:
  void find(Mat& schene);
  void learn(Mat& schene, Rect& boundary);
  
  void onFrame(Mat& frame);
  void onKey(char key);
};

#endif