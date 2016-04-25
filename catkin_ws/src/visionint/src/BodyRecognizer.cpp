#include "dirent.h"
#include <cstdio>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Eigen>
#include "opencv2/core/eigen.hpp"

#include "BodyRecognizer.h"
#include "vision.h"


BodyRecognizer::BodyRecognizer(FeedbackPublisher* publisher) : BodyDetector()
{
  feedbackPublisher = publisher;
  readBodies();
}


float BodyRecognizer::calculateDistance(cv::Mat& d1, cv::Mat& d2)
{
  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix4f> ges;
  Eigen::Matrix4f d1Eigen;
  Eigen::Matrix4f d2Eigen;
  
  cv::cv2eigen(d1, d1Eigen); 
  cv::cv2eigen(d2, d2Eigen); 
  
  ges.compute(d1Eigen, d2Eigen);
  
  double sum;
  for(int i=0; i < ges.eigenvalues().size(); i++)
  {
    sum += pow(log(ges.eigenvalues()[i]), 2.0);
  }
  double d = sqrt(sum);
  //cout << "The generalzied eigenvalues are : " << ges.eigenvalues().transpose() << endl;
  std::cout << "distance is " << d << std::endl;
  return d;
}




void BodyRecognizer::readBodies()
{
  std::string filenamePrefix = "person";
  std::cout << "reading bodies from " << BODIES_DIRECTORY << std::endl;
  knownBodies.clear();
  
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (BODIES_DIRECTORY.c_str())) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      std::string filename(ent->d_name);
      if(filename.compare(0, filenamePrefix.length(), filenamePrefix) == 0){
        std::cout << "body file: " << filename << std::endl;
        std::string personName = filename.substr(0, filename.length() - 4); //drop .yml
        std::cout << "person name: " << personName << std::endl;
        
        cv::FileStorage fs(BODIES_DIRECTORY + "/" + filename, cv::FileStorage::READ);
        cv::Mat personDescriptor;
        
        fs[personName] >> personDescriptor;
        
        knownBodies[personName] = personDescriptor;
        
      }
    }
    closedir (dir);
  } else {
    std::cerr << "couldn't read directory " << BODIES_DIRECTORY << std::endl;
    return;
  }
}


void BodyRecognizer::onFrame(cv::Mat& frame)
{
  recognize(frame);
}

void BodyRecognizer::onKey(char key)
{
  
}


void BodyRecognizer::recognize(cv::Mat& frame)
{
  std::vector< cv::Rect_<int> > bodies;
  bodies = detectBodies(frame);
  if(bodies.size() == 1) {
    cv::Mat body = frame(bodies[0]);
    cv::Mat d = getDescriptor(body);
    std::string recognized = findNearest(d);
    std::cout << "person is " << recognized << std::endl;
    if(recognized != "unknown") {
      cv::Rect b = bodies[0];
      int x = (b.tl().x + b.br().x) / 2;
      int y = (b.tl().y + b.br().y) / 2;
      feedbackPublisher->publishDetectedBody(recognized, x, y);
    }
      
  }
}


std::string BodyRecognizer::findNearest(cv::Mat& descriptor)
{
  //TODO: move maximumDistance to constants
  float maximumDistance = 3.0;
  
  std::string name = "unknown";
  float distance = INFINITY;
  
  for (std::map<std::string, cv::Mat>::iterator it = knownBodies.begin(); it != knownBodies.end(); ++it)
  {
    float d = calculateDistance(it->second, descriptor);
    if(d < distance && d < maximumDistance){
      name = it->first;
      distance = d;
      std::cout << name << " is nearest" << std::endl;
    }
  }
  return name;
}
