#ifndef VISION_NODE_H_
#define VISION_NODE_H_

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <image_transport/image_transport.h>

// Custom message includes. Auto-generated from msg/ directory.
#include "visionint/RecoGoal.h"
#include "visionint/RecoResults.h"
#include "std_msgs/Int32.h"

#include "src/FindBody.h"
#include "src/FaceRecog.h"
#include "src/FaceRecognizerTeacher.h"
#include "src/FaceDetector.h"
#include "src/FeedbackPublisher.h"

#include "src/BodyRecognizer.h"
#include "src/BodyRecognizerTeacher.h"
#include "src/FindObject.h"



class VisionNode : public FeedbackPublisher
{
  FaceRecog* faceRecognizer;
  FaceRecognizerTeacher* faceRecognizerTeacher;
  FaceDetector* faceDetector;
  BodyRecognizer* bodyRecognizer;
  BodyRecognizerTeacher* bodyRecognizerTeacher;
  FindObject* findObject;
  
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
  
  void publishFeedback(visionint::RecoResults* results);
  
  void messageCallback(const std_msgs::Int32::ConstPtr& msg);
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif // VISION_NODE_H_
