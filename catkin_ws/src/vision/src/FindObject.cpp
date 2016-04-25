#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <opencv2/opencv_modules.hpp>
#include <opencv2/features2d/features2d.hpp>

#ifdef HAVE_OPENCV_NONFREE
  #if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >=4
  #include <opencv2/nonfree/gpu.hpp>
  #include <opencv2/nonfree/features2d.hpp>
  #endif

#endif

#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif

#include "FindObject.h"


using namespace cv;

void FindObject::onKey()
{
  
}


void FindObject::onFrame(Mat& frame)
{

}


static void FindObject::onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        //selection &= Rect(0, 0, image.cols, image.rows);
    }
    
    
    switch( event )
    {
      case CV_EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
      case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 ){
          objectSelected = true;
        }
        break;
    }
}



void FindObject::learn(Mat& frame)
{
  objectImg = frame(selection);
  cvtColor(objectImg, objectImg, CV_BGR2GRAY);    
  
  string capturedImageWindow = "Captured";
  imshow(capturedImageWindow, objectImg);
  waitKey(1000);
  destroyWindow(capturedImageWindow);
  
  detector = cv::Ptr<cv::FeatureDetector>(new cv::SIFT());
  std::cout << "find 1" << std::endl;

  detector->detect(objectImg, objectKeypoints);
  printf("Object: %d keypoints detected  ", (int)objectKeypoints.size());

  extractor = cv::Ptr<cv::DescriptorExtractor>(cv::FeatureDetector::create("SIFT"));
  
  
  extractor->compute(objectImg, objectKeypoints, objectDescriptors);
  
  printf("Object: %d descriptors extracted ", objectDescriptors.rows);
  
  
  objectSelected = false;
  if(objectDescriptors.rows > 0) {
    std::cout << "object learned" << std::endl;
    objectLearned = true;
  }
}


void FindObject::find(Mat& frame)
{
  Mat sceneImg;
  cv::cvtColor(frame, sceneImg, CV_BGR2GRAY);
  std::cout << "find 1" << std::endl;

  detector->detect(sceneImg, sceneKeypoints);
  printf("Scene: %d keypoints detected ", (int)sceneKeypoints.size());
  std::cout << "find 2" << std::endl;

  extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
  printf("Scene: %d descriptors extracted", sceneDescriptors.rows);

  cv::Mat results;
  cv::Mat dists;
  std::vector<std::vector<cv::DMatch> > matches;
  int k=2; // find the 2 nearest neighbors
  bool useBFMatcher = false;
  if(objectDescriptors.type()==CV_8U)
  {

    printf("Binary descriptors detected...\n");
    if(useBFMatcher)
    {
      cv::BFMatcher matcher(cv::NORM_HAMMING);
      matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
    }
    else
    {
      cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
      printf("Time creating FLANN LSH index");
      
      flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
    }
  }
  else
  {
    printf("Float descriptors detected...\n");
    if(useBFMatcher)
    {
      cv::BFMatcher matcher(cv::NORM_L2);
      matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
    }
    else
    {
      cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
      printf("Time creating FLANN KDTree index ");
      
      flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
    }
  }
  printf("Time nearest neighbor search ");
  
  if(dists.type() == CV_32S)
  {
    cv::Mat temp;
    dists.convertTo(temp, CV_32F);
    dists = temp;
  }
  
  
  float nndrRatio = 0.8f;
  std::vector<cv::Point2f> mpts1, mpts2; // Used for homography
  std::vector<int> indexes1, indexes2; // Used for homography
  std::vector<uchar> outlierMask;  // Used for homography
  // Check if this descriptor matches with those of the objects
  if(!useBFMatcher)
  {
    for(int i=0; i<objectDescriptors.rows; ++i)
    {
      if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
        dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
      {
        mpts1.push_back(objectKeypoints.at(i).pt);
        indexes1.push_back(i);
        
        mpts2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
        indexes2.push_back(results.at<int>(i,0));
      }
    }
  }
  else
  {
    for(unsigned int i=0; i<matches.size(); ++i)
    {
      if(matches.at(i).size() == 2 &&
        matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
      {
        mpts1.push_back(objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
        indexes1.push_back(matches.at(i).at(0).queryIdx);
        
        mpts2.push_back(sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
        indexes2.push_back(matches.at(i).at(0).trainIdx);
      }
    }
  }
  
  unsigned int minInliers = 8;
  if(mpts1.size() >= minInliers)
  {
    cv::Mat H = findHomography(mpts1,
                               mpts2,
                               cv::RANSAC,
                               1.0,
                               outlierMask);
    printf("Time finding homography");
    
    
    std::vector<Point2f> objCorners(4);
    objCorners[0] = cvPoint(0,0);
    objCorners[1] = cvPoint( objectImg.cols, 0 );
    objCorners[2] = cvPoint( objectImg.cols, objectImg.rows );
    objCorners[3] = cvPoint( 0, objectImg.rows );
    std::vector<Point2f> sceneCorners(4);
    
    perspectiveTransform(objCorners, sceneCorners, H);
    
    line( frame, sceneCorners[0], sceneCorners[1], Scalar(0, 255, 0), 4 );
    line( frame, sceneCorners[1], sceneCorners[2], Scalar( 0, 255, 0), 4 );
    line( frame, sceneCorners[2], sceneCorners[3], Scalar( 0, 255, 0), 4 );
    line( frame, sceneCorners[3], sceneCorners[0], Scalar( 0, 255, 0), 4 );
    
    
    imshow(win, frame);
    
    
  }
  else
  {
    printf("Not enough matches (%d) for homography...\n", (int)mpts1.size());
  }
  
}


int main(int argc, char *argv[])
{

  int deviceId = CV_CAP_ANY;
  VideoCapture capture(deviceId);
  if(!capture.isOpened()) {
    std::cerr << "Capture Device ID " << deviceId << "cannot be opened." << std::endl;
    return -1;
  }
  
  namedWindow(win, CV_WINDOW_NORMAL);
  
  cvSetMouseCallback(win, onMouse, 0);
  
  
  Mat frame;
  for(;;)
  {
    capture >> frame;
    char key = (char) waitKey(20);
    
    if(objectSelected){
      std::cout << "learing object" << std::endl;
      learnObject(frame);
    } else if(objectLearned) {
      findObject(frame);
    } else if(selectObject) {
      if( selection.width > 0 && selection.height > 0 ){
        rectangle(frame, selection, CV_RGB(255, 0, 0));
      }
      imshow(win, frame);
    } else {
      imshow(win, frame);
    }
    
    
    if(key == ' '){
      std::cout << "find object" << std::endl;
      findObject(frame);
      break;
    } else if(key == 'e'){
      exit(0);
      break;
    }
  }
  
  printf("Closing...\n");
  
  return 0;
}
