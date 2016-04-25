#ifndef BODYRECOGNIZERTEACHER_H
#define BODYRECOGNIZERTEACHER_H

#include <src/BodyDetector.h>
#include <src/FeedbackPublisher.h>


class BodyRecognizerTeacher : public BodyDetector
{
  uint savedPersonIdx = 0;
  bool learning = false;
    
  void detect(cv::Mat& frame);
  void saveDescriptor(cv::Mat& descriptor);

  FeedbackPublisher* feedbackPublisher;
  
public:
    BodyRecognizerTeacher(FeedbackPublisher* feedbackPublisher);
    
    void startLearningBody();
    virtual void onFrame(cv::Mat& frame);
    virtual void onKey(char key);
};

#endif // BODYRECOGNIZERTEACHER_H
