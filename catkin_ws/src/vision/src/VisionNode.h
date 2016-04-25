#ifndef VISION_NODE_H_
#define VISION_NODE_H_

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <image_transport/image_transport.h>

// Custom message includes. Auto-generated from msg/ directory.
#include "vision/RecoGoal.h"
#include "vision/RecoResults.h"

#include "src/FindBody.h"
#include "src/FaceRecog.h"
#include "src/FaceRecognizerTeacher.h"
#include "src/FaceDetector.h"
#include "src/FeedbackPublisher.h"

#include "src/BodyRecognizer.h"
#include "src/BodyRecognizerTeacher.h"



class VisionNode : public FeedbackPublisher
{
  FaceRecog* faceRecognizer;
  FaceRecognizerTeacher* faceRecognizerTeacher;
  FaceDetector* faceDetector;
  BodyRecognizer* bodyRecognizer;
  BodyRecognizerTeacher* bodyRecognizerTeacher;
  
  Processor* current;
  
  bool isActive = false;
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber imgSubscriber;
  image_transport::Publisher imgPublisher;
    
  ros::Publisher feedbackPublisher;
  ros::Subscriber messageSubscriber;
  
public:
  VisionNode();
  
  ~VisionNode();
  
  void publishFeedback(vision::RecoResults* results);
  
  void messageCallback(const vision::RecoGoal::ConstPtr& msg);
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif // VISION_NODE_H_
