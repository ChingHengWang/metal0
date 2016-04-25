#ifndef FINDBODY_H_
#define FINDBODY_H_

#include "processor.h"


class FindBody : public Processor
{
private:
  cv::Mat res1;
  cv::Mat frameDelta;
  cv::Mat avg;
  cv::Mat gray;
  bool isBackgroundInitialized = false;
  
public:
    virtual void onFrame(cv::Mat& frame);
    virtual void onKey(char key);
    void printCommand(cv::Mat& frame, cv::Rect& faceCenter);
};


#endif /* FINDBODY_H_ */
