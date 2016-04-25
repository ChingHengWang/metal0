#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#include <opencv2/core/core.hpp>


class Processor
{

public:
    virtual void onFrame(cv::Mat& frame){};
    virtual void onKey(char key){};

};


#endif /* PROCESSOR_H_ */
