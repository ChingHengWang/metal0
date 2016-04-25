#include <iostream>
#include <fstream>
#include <sstream>

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
  #include <opencv2/nonfree/features2d.hpp>
  #endif
#endif

#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif

#include "ros/ros.h"

#include "vision.h"
#include "FindObject.h"


using namespace cv;

void FindObject::onKey(char key)
{
  
}


void FindObject::onFrame(Mat& frame)
{
  find(frame);
}


void FindObject::readCsv(const string& basedir, vector<Mat>& images, char separator) {
  std::string filename = basedir + "objects_info.csv";
  std::cout << "reading object descriptions from " << filename << std::endl;
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file) {
        ROS_ERROR("couldn't load object info from %s", filename.c_str());
    }
    string line, path, classlabel;
    Mat img;
    while (getline(file, line)) {
      std::stringstream liness(line);
      std::getline(liness, classlabel, separator);
      std::getline(liness, path);
      if(!path.empty() && !classlabel.empty()) {
        img = imread(basedir + path, 0); // 0 means load gray image
        if(!img.empty()){
          std::cout << "loaded " << classlabel << " from " << basedir << path << std::endl;
          images.push_back(img.clone());
          labels.push_back(classlabel);
        } else {
          ROS_ERROR("couldn't load image from %s", (basedir + path).c_str());
        }
      }
    }
}


void FindObject::removeBackground(Mat& image)
{
}



void FindObject::calculateDescriptors(std::vector<cv::Mat>& images)
{
  for(uint i = 0; i < images.size(); i++)
  {
    Mat objectImg = images[i];
    //cvtColor(objectImg, objectImg, CV_BGR2GRAY); 
    
    //detector = cv::Ptr<cv::FeatureDetector>(new cv::SIFT());
    detector = cv::Ptr<cv::FeatureDetector>(cv::FeatureDetector::create("SIFT"));
    
    detector->detect(objectImg, objectKeypoints);
    
    printf("%s: %d keypoints detected  ", labels[i].c_str(), (int)objectKeypoints.size());
    
    extractor = cv::Ptr<cv::DescriptorExtractor>(cv::FeatureDetector::create("SIFT"));
    
    
    extractor->compute(objectImg, objectKeypoints, objectDescriptors);
    
    objectsKeypoints.push_back(objectKeypoints);
    objectsDescriptors.push_back(objectDescriptors);
    
    printf("%d descriptors extracted\n", objectDescriptors.rows);
    
    fflush(stdout);
    
  }
  
}


void FindObject::find(Mat& frame)
{
  Mat sceneImg;
  cv::cvtColor(frame, sceneImg, CV_BGR2GRAY);
  
  detector->detect(sceneImg, sceneKeypoints);
  //printf("Scene: %d keypoints detected ", (int)sceneKeypoints.size());
  
  extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
  //printf("Scene: %d descriptors extracted", sceneDescriptors.rows);
  
  for(uint i = 0; i < labels.size(); i++)
  {
    objectKeypoints = objectsKeypoints[i];
    objectDescriptors = objectsDescriptors[i];
    objectImg = images[i];
    
    cv::Mat results;
    cv::Mat dists;
    std::vector<std::vector<cv::DMatch> > matches;
    int k=2; // find the 2 nearest neighbors
    bool useBFMatcher = false;
    if(objectDescriptors.type()==CV_8U)
    {
      
      //printf("Binary descriptors detected...\n");
      if(useBFMatcher)
      {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
      }
      else
      {
        cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
        
        flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
      }
    }
    else
    {
      if(useBFMatcher)
      {
        cv::BFMatcher matcher(cv::NORM_L2);
        matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
      }
      else
      {
        cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
      }
    }
    
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
      
      std::cout << "detected " << labels[i] << std::endl;
      
      int x = 0;
      int y = 0;
      
      for(uint i = 0; i < sceneCorners.size(); i++)
      {
        x += sceneCorners[i].x;
        y += sceneCorners[i].y;
      }
      x = x / sceneCorners.size();
      y = y / sceneCorners.size();
      
      feedbackPublisher->publishDetectedObject(labels[i], x, y);
      
    }
    else
    {
      //printf("Not enough matches (%d) for homography...\n", (int)mpts1.size());
    }
    
  }
}


FindObject::FindObject(FeedbackPublisher* publisher)
{
  feedbackPublisher = publisher;
  readCsv(OBJECTS_DIRECTORY, images);
  calculateDescriptors(images);
}

