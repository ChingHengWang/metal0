#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include "FaceRecog.h"
#include "vision.h"
#include "visionint/RecoResults.h"


using namespace cv;
using std::cout;
using std::endl;

  
const char* FaceRecog::WINDOW_NAME = "CV";

  
void FaceRecog::onFrame(Mat& frame)
{
  startDetection(frame);
}


void FaceRecog::onKey(char key)
{
    switch (key) 
    {
      case ' ':
      break;
      case 'r':
        cout << "starting face recognition" << endl;
        break;
    case 'i':
      increaseRecognizerThreshold(1.5);
      break;
    case 'd':
      increaseRecognizerThreshold(0.7);
      break;
    case 'c':
      tracking = false;
      break;
    case 27: //ESC
      std::cout << "end tracking" << std::endl;
      exit(0);
    default:
        break;
    }
}


void FaceRecog::loadFaces()
{
  faceRecognizer->load(FACES_FILENAME);
}


void FaceRecog::initFaceRecognizer()
{
  double initialThreshold = 50000.0;
  faceRecognizer = createEigenFaceRecognizer(0, initialThreshold);
}


void FaceRecog::increaseRecognizerThreshold(double inc)
{
  double increased = inc * faceRecognizer->getDouble("threshold");
  faceRecognizer->set("threshold", increased);
  std::cout << "current threshold is " << increased << std::endl;
}


void FaceRecog::initTracking(Rect face, Mat& f)
{
  selection = face;
  
  int _vmin = vmin, _vmax = vmax;
  
  inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
          Scalar(180, 256, MAX(_vmin, _vmax)), mask);
  int ch[] = {0, 0};
  hue.create(hsv.size(), hsv.depth());
  mixChannels(&hsv, 1, &hue, 1, ch, 1);
  
  Mat roi(hue, selection), maskroi(mask, selection);
  calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
  normalize(hist, hist, 0, 255, CV_MINMAX);
  
  trackWindow = selection;
  
  histimg = Scalar::all(0);
  int binW = histimg.cols / hsize;
  Mat buf(1, hsize, CV_8UC3);
  for( int i = 0; i < hsize; i++ )
    buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
  cvtColor(buf, buf, CV_HSV2BGR);
  
  for( int i = 0; i < hsize; i++ )
  {
    int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
    rectangle( histimg, Point(i*binW,histimg.rows),
               Point((i+1)*binW,histimg.rows - val),
               Scalar(buf.at<Vec3b>(i)), -1, 8 );
  }
  
  showRoi(f);
  tracking = true;
}


void FaceRecog::track(Mat& frame)
{
  int _vmin = vmin, _vmax = vmax;
  
  inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
          Scalar(180, 256, MAX(_vmin, _vmax)), mask);
  int ch[] = {0, 0};
  hue.create(hsv.size(), hsv.depth());
  mixChannels(&hsv, 1, &hue, 1, ch, 1);
  
  calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
  backproj &= mask;
  RotatedRect trackBox = CamShift(backproj, trackWindow,
                                  TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 1 ));
  
  if( trackWindow.area() <= 1 )
  {
    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                       trackWindow.x + r, trackWindow.y + r) &
                       Rect(0, 0, cols, rows);
  }
  
  ellipse(frame, trackBox, Scalar(0,0,255), 3, CV_AA );
  drawArrow(frame, trackBox.center);
  printCommand(frame, trackBox.center);
}


void FaceRecog::showRoi(Mat& frame)
{
  Mat roi(frame, selection);
  bitwise_not(roi, roi);
  imshow("face_recognizer", frame);
  waitKey(1000);
}


void FaceRecog::drawArrow(Mat& frame, Point to)
{
  int arrowMagnitude = 9;
  int centerX = frame.cols / 2;
  int centexY = frame.rows / 2;
  const double PI = 3.141592653;
  Point from = Point(centerX, centexY);
  
  double angle = atan2((double)from.y-to.y, (double)from.x-to.x);
  
  line(frame, from, to, CV_RGB(255, 0, 0));
  
  from.x = (int) ( to.x + arrowMagnitude * cos(angle + PI/4));
  from.y = (int) ( to.y + arrowMagnitude * sin(angle + PI/4));
  line(frame, to, from, CV_RGB(255, 0, 0));
  
  from.x = (int) ( to.x + arrowMagnitude * cos(angle - PI/4));
  from.y = (int) ( to.y + arrowMagnitude * sin(angle - PI/4));
  line(frame, to, from, CV_RGB(255, 0, 0));
  
}


void FaceRecog::printCommand(Mat& frame, Point faceCenter)
{
  
  int positionrl = (int) 100 * ((double)(frame.cols / 2 - faceCenter.x) / (frame.cols / 2));
  int positionud = (int) 100 * ((double)(frame.rows / 2 - faceCenter.y) / (frame.rows / 2));
  
  //TODO: personId?
  std::cout << "{\"mode\": " << "facedetector" <<  ", " << "\"id\": " << 0 << ", ";
  
  if(positionrl > 0){
    cout << "\"left\": "  <<positionrl <<", ";
  } else {
    cout << "\"right\": " << std::abs(positionrl) <<", ";
  }
  
  
  if(positionud > 0){
    std::cout << "\"up\": "  << std::abs(positionud);
  } else {
    std::cout << "\"down\": " << std::abs(positionud);
  }
  
  cout << "}" << std::endl;
  
}


void FaceRecog::predict(Mat& original)
{
  Mat gray;
  cvtColor(original, gray, CV_BGR2GRAY);
  
  vector< Rect_<int> > faces = detectFaces(gray);
  
  
  for(uint i = 0; i < faces.size(); i++) {
    Rect faceRect = faces[i];
    Mat face = gray(faceRect);
    Mat face_resized;
    cv::resize(face, face_resized, Size(FACE_WIDTH, FACE_HEIGHT), 1.0, 1.0, INTER_CUBIC);
    int prediction = faceRecognizer->predict(face_resized);
    if(prediction != -1) {
      rectangle(original, faceRect, CV_RGB(0, 255,0), 1);
      string box_text = format("This is = #%d person", prediction + 1);
      int pos_x = std::max(faceRect.tl().x - 10, 0);
      int pos_y = std::max(faceRect.tl().y - 10, 0);
      putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
      int faceX = (faceRect.tl().x + faceRect.br().x) / 2;
      int faceY = (faceRect.tl().y + faceRect.br().y) / 2;
      feedbackPublisher->publishDetectedPerson(prediction + 1, faceX, faceY);
    }
  }
}


void FaceRecog::startDetection(Mat frame) {
  
  cvtColor(frame, hsv, COLOR_BGR2HSV);
  
  if(!tracking) {
    predict(frame);
  } else {
    track(frame);
  }
  
  //imshow(WINDOW_NAME, original);
  
}




FaceRecog::FaceRecog(FeedbackPublisher* p) : FaceDetector() {
  feedbackPublisher = p;
  histimg = Mat::zeros(200, 320, CV_8UC3);
  phranges = hranges;
  
  initFaceRecognizer();
}




