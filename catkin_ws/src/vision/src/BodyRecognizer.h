#ifndef BODYRECOGNIZER_H
#define BODYRECOGNIZER_H

#include <src/BodyDetector.h>
#include <src/FeedbackPublisher.h>


class BodyRecognizer : public BodyDetector
{
  std::map<std::string, cv::Mat> knownBodies;
  void readBodies();
  float calculateDistance(cv::Mat& d1, cv::Mat& d2);
  void recognize(cv::Mat& frame);
  std::string findNearest(cv::Mat& descriptor);
  
  FeedbackPublisher* feedbackPublisher;
  
public:
    BodyRecognizer(FeedbackPublisher* feedbackPublisher);
    virtual void onFrame(cv::Mat& frame);
    virtual void onKey(char key);
};

#endif // BODYRECOGNIZER_H
