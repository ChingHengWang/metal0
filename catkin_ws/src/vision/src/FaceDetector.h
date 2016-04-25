#ifndef FACEDETECTOR_H_
#define FACEDETECTOR_H_

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "processor.h"


class FaceDetector : public Processor
{
  static const char * HAAR_FRONTALFACE_DEFAULT_PATH;
  static const char * HAAR_FRONTALFACE_ALT_PATH;
  
  static const int MIN_FACE_HEIGHT = 150;
  static const int MIN_FACE_WIDTH = 150;
  
  cv::CascadeClassifier haarCascade;
  cv::Ptr<cv::FaceRecognizer> faceRecognizer;
  
  
  void loadHaarCascades(const char* filename);
  
protected:
  static const int FACE_WIDTH = 200;
  static const int FACE_HEIGHT = 200;
  
  std::vector< cv::Rect_<int> > detectFaces(cv::Mat& gray);
  
  
public:
  FaceDetector();
  void onFrame(cv::Mat& frame);
  void onKey(char key);
};

#endif /* FACEDETECTOR_H_ */