#include "VisionNode.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


//TODO: unsubscribe from camera topic in inactive state
VisionNode::VisionNode() : it(nh)
{
  ROS_INFO("starting vision node");
  messageSubscriber = nh.subscribe("/andbot/fr_order", 1000, &VisionNode::messageCallback, this);
  feedbackPublisher = nh.advertise<vision::RecoResults>("/andbot/feedback", 10);
  
  imgSubscriber = it.subscribe("/camera/image_raw", 1, &VisionNode::imageCallback, this);
  imgPublisher = it.advertise("/camera/image_raw/result", 1);
  
  faceRecognizerTeacher = new FaceRecognizerTeacher(this);
  faceRecognizer = new FaceRecog(this);
  faceDetector = new FaceDetector();
  
  bodyRecognizer = new BodyRecognizer(this);
  bodyRecognizerTeacher = new BodyRecognizerTeacher(this);
}

VisionNode::~VisionNode()
{
  ROS_INFO("stopping vision node");
}

void VisionNode::publishFeedback(vision::RecoResults* results)
{
  ROS_INFO("pubilising results");
  feedbackPublisher.publish(*results);
  delete(results);
}

void VisionNode::messageCallback(const vision::RecoGoal::ConstPtr &msg)
{
  ROS_INFO("received order_id=%d, type_reco=%d", msg->order_id, msg->type_reco);
  switch(msg->order_id)
  {
    case 1:
      switch(msg->type_reco)
      {
        case 1:
          ROS_INFO("starting face recognizer training");
          current = faceRecognizerTeacher;
          isActive = true;
          break;
        case 2:
          ROS_INFO("starting body recognizer training");
          current = bodyRecognizerTeacher;
          isActive = true;
          break;
      }
      break;
        case 2:
          //update database
          break;
        case 3:
          switch(msg->type_reco)
          {
            case 1:
              ROS_INFO("starting face recognizer");
              faceRecognizer->loadFaces();
              current = faceRecognizer;
              isActive = true;
              break;
            case 2:
              ROS_INFO("starting body recognizer");
              current = bodyRecognizer;
              isActive = true;
              break;
            case 3:
              //findObject
              break;
          }
          //stop reco
          break;
            case 4:
              isActive = false;
              break;
  }
  
}


void VisionNode::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
  if(isActive) {
    ROS_DEBUG("got image!");
    cv_bridge::CvImagePtr cv_ptr;
    //cv_bridge::CvImageConstPtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      //cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    current->onFrame(cv_ptr->image);
    
    //cv::imshow("win", cv_ptr->image);
    
    imgPublisher.publish(cv_ptr->toImageMsg());
  }
}
