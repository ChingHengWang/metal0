#include <iostream>
#include "vision.h"

#include "BodyRecognizerTeacher.h"

BodyRecognizerTeacher::BodyRecognizerTeacher(FeedbackPublisher* publisher) : BodyDetector()
{
  feedbackPublisher = publisher;
}

void BodyRecognizerTeacher::onFrame(cv::Mat& frame)
{
  detect(frame);
}

void BodyRecognizerTeacher::onKey(char key)
{
  
}


void BodyRecognizerTeacher::startLearningBody()
{
  learning = true;
}



void BodyRecognizerTeacher::detect(cv::Mat& frame)
{
  std::vector< cv::Rect_<int> > bodies;
  bodies = detectBodies(frame);
  if(learning && bodies.size() == 1) {
    cv::Mat body = frame(bodies[0]);
    cv::Mat d = getDescriptor(body);
    saveDescriptor(d);
    learning = false;
  }
}


void BodyRecognizerTeacher::saveDescriptor(cv::Mat& descriptor)
{
  std::string personName = "person_" + std::to_string(savedPersonIdx++);
  auto filename = BODIES_DIRECTORY + "/" + personName + ".yml";
  std::cout << "saving person description to " << filename << std::endl;
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  file << personName << descriptor;
}
