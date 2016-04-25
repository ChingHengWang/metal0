#ifndef FACE_RECOGNIZER_TEACHER_H
#define FACE_RECOGNIZER_TEACHER_H

#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>

#include "src/processor.h"
#include "src/FaceDetector.h"
#include "src/FeedbackPublisher.h"


class FaceRecognizerTeacher : public FaceDetector
{
  const static int MAX_PERSON_SAMPLES = 5;
  const static int MIN_FRAMES_BEETWEN_SAMPLES = 10;
  const static char* WINDOW_NAME;
  
  bool learningFace = false;
  
  std::vector<cv::Mat> samples;
  int currentLabel = 0;
  std::vector<int> labels;
  
  int takenPersonSamples = 0;
  int skippedFrames = 0;
  
  cv::Ptr<cv::FaceRecognizer> faceRecognizer;
  
  FeedbackPublisher* feedbackPublisher;
  
  void loadPredefinedImages();
  void findFace(cv::Mat& frame, cv::Mat& result);
  void learnFace(cv::Mat& frame);
  void save();
  
public:
  FaceRecognizerTeacher(FeedbackPublisher* feedbackPublisher);
  
  void startLearnigPerson();
  
  virtual void onFrame(cv::Mat& frame);
  virtual void onKey(char key);
};

#endif // FACE_RECOGNIZER_TEACHER_H
