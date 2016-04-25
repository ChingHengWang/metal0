#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include "vision.h"
#include "FindBody.h"
#include "FaceDetector.h"
#include "FaceRecog.h"
#include "FaceRecognizerTeacher.h"
#include "VisionNode.h"



int main(int argc, char **argv) {
    
  // Set up ROS.
  ros::init(argc, argv, "vision");

  VisionNode node;
  
  ros::spin();
  
  return 0;
    
      
//    for(;;)
//    {
//        capture >> frame;
//        Mat c = frame.clone();
//        char k  = cv::waitKey(20);
//        
//        switch(k)
//        {
//          case 'd':
//            facerecognizer->loadFaces();
//            current = facerecognizer;
//            default:
//              current->onKey(k);
//        }
//        
//        current->onFrame(c);
//    }
//
//  return 0;
}