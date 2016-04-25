#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ros/ros.h"

#include "FaceRecognizerTeacher.h"
#include "vision.h"



using std::cout;
using std::endl;

using namespace cv;

const char* FaceRecognizerTeacher::WINDOW_NAME = "CV";

void FaceRecognizerTeacher::onFrame(Mat& frame)
{
    learnFace(frame);
}


void FaceRecognizerTeacher::startLearnigPerson()
{
  if(!learningFace){
    learningFace = true;
  }
}



void FaceRecognizerTeacher::onKey(char key)
{
    switch(key)
    {
      case ' ':
        learningFace = true;
        break;
      case 's':
        save();
        break;
      case 27: //ESC
        exit(0);
    }
}


void FaceRecognizerTeacher::save()
{
  cout << "saving faces to " << FACES_FILENAME << endl;
  faceRecognizer->train(samples, labels);
  faceRecognizer->save(FACES_FILENAME);
}


void FaceRecognizerTeacher::loadPredefinedImages()
{
  cout << "loading predefinig images" << endl;
  char filename[100] = {0};
  Mat face;
  for(uint i = 0; i < 10; i++)
  {
    //cwd is src
    std::sprintf(filename, "../predefined/person_%.3d.jpg", i);
    cout << "loading " << filename << endl;
    face = imread(filename);
    if(!face.empty()) {
      cout << "read success" << endl;
    } else {
      cout << "couldn't load " << filename << endl;
    }
    samples.push_back(face.clone());
    labels.push_back(100500); //this person shouldn't be found
  }
}


void FaceRecognizerTeacher::findFace(Mat& frame, Mat& result) 
{
  Mat gray;
  cvtColor(frame, gray, CV_BGR2GRAY);
  vector< Rect_<int> > faces = detectFaces(gray);
  if(faces.size() == 1) {
    Rect f = faces[0];
    Mat face = gray(f);
    cout << "found face height: " << face.rows << ", width: " << face.cols << endl;
    feedbackPublisher->publishFoundFace(f.x, f.y);
    cv::resize(face, result, Size(FACE_WIDTH, FACE_HEIGHT), 1.0, 1.0, INTER_CUBIC);
  } 
}


void FaceRecognizerTeacher::learnFace(Mat& frame) 
{
  
  vector< Rect_<int> > faces = detectFaces(frame);
  
  //show detected faces
  for(uint i = 0; i < faces.size(); i++) {
    rectangle(frame, faces[i], Scalar(0,255,0), 3);
  }
  //cout << faces.size() << " faces detected" << endl;
  //imshow(WINDOW_NAME, frame);
  
  
  //try to take sample
  if(learningFace) {
    if(skippedFrames >= MIN_FRAMES_BEETWEN_SAMPLES || takenPersonSamples == 0){
      Mat face;
      findFace(frame, face);
      if(!face.empty()){
        takenPersonSamples++;
        skippedFrames = 0;
        std::cout << "saving image for " << currentLabel << " person" << std::endl;
        samples.push_back(face);
        labels.push_back(currentLabel);
      } else {
        ROS_DEBUG("face not found");
      }
    } else {
      skippedFrames++;
      //std::cout << "skipping frame " << skippedFrames << std::endl;
    }
  }
  
  
  if(takenPersonSamples == MAX_PERSON_SAMPLES) {
    //next person
    feedbackPublisher->publishSavedPerson(currentLabel);
    save();
    learningFace = false;
    currentLabel += 1;
    takenPersonSamples = 0;
    skippedFrames = 0;
  }
}



//void FaceRecognizerTeacher::displayFoundFace(Mat& face)
//{
//  string capturedImageWindow = "Captured";
//  imshow(capturedImageWindow, face);
//  waitKey(1000);
//  destroyWindow(capturedImageWindow);
//}



FaceRecognizerTeacher::FaceRecognizerTeacher(FeedbackPublisher* publisher) : FaceDetector()
{
  feedbackPublisher = publisher;
  double initialThreshold = 50000.0;
  faceRecognizer = cv::createEigenFaceRecognizer(0, initialThreshold);
}



