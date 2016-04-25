#ifndef FEEDBACK_PUBLISHER_H
#define FEEDBACK_PUBLISHER_H

#include "visionint/RecoResults.h"


class FeedbackPublisher 
{
protected:  
  virtual void publishFeedback(visionint::RecoResults* results){};
public:
  void publishFoundFace(int x, int y);
  void publishSavedPerson(int n);
  void publishDetectedPerson(int idx, int x, int y);
  
  void publishDetectedBody(std::string name, int x, int y);

  void publishDetectedObject(std::string name, int x, int y);
};


#endif /* FEEDBACK_PUBLISHER_H */
