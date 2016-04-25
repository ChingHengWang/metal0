#ifndef FIND_OBJECT_H_
#define FIND_OBJECT_H_

#include "processor.h"
#include "src/FeedbackPublisher.h"

class FindObject : public Processor {
  cv::Point origin;
  cv::Rect selection;
  
  std::vector<cv::KeyPoint> objectKeypoints;
  cv::Mat objectDescriptors;
  cv::Mat objectImg;
  
  std::vector<cv::KeyPoint> sceneKeypoints;
  cv::Mat sceneDescriptors;
  
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  
  std::vector<std::string> labels;
  std::vector<cv::Mat> images;
  std::vector< std::vector<cv::KeyPoint> > objectsKeypoints;
  std::vector<cv::Mat> objectsDescriptors;
  
  FeedbackPublisher* feedbackPublisher;
  
  void find(cv::Mat& schene);
  void learn(cv::Mat& schene);
  void readCsv(const std::string& filename, std::vector<cv::Mat>& images, char separator = ',');
  void removeBackground(cv::Mat& image);
  void calculateDescriptors(std::vector<cv::Mat>& images);
  
public:
  FindObject(FeedbackPublisher* feedbackPublisher);
  
  void onFrame(cv::Mat& frame);
  void onKey(char key);
};

#endif