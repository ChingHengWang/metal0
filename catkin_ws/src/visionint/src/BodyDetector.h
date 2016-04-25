#ifndef BODYDETECTOR_H
#define BODYDETECTOR_H

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "processor.h"


class BodyDetector : public Processor
{
    static const char* HAAR_FULLBODY_PATH;

    static const int MIN_BODY_HEIGHT = 150;
    static const int MIN_BODY_WIDTH = 100;

    static const int BODY_HEIGHT = 100;
    static const int BODY_WIDTH = 100;
    
    cv::CascadeClassifier haarCascade;

    void loadHaarCascades(const char* filename);
    
protected:
  void calculateCovar(cv::Mat& input, cv::Mat& result);
  cv::Mat getDescriptor(cv::Mat& roi);

public:
    BodyDetector();
    std::vector< cv::Rect_<int> > detectBodies(cv::Mat& frame);
    void onFrame(cv::Mat& frame);
    void onKey(char key);
};

#endif /* BODYDETECTOR_H */
