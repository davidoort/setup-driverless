#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <math.h>
#include <cmath>

using namespace cv;
using namespace cv::ml;

#include "ros/ros.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"

using namespace std;


class TrackFinder {
public:
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber;
  ros::Publisher publisher;

  TrackFinder() {
    // Initialize the publisher on the '/car/targetline' topic
    publisher = nodeHandle.advertise<track::Line>("/car/targetline", 1000);
  }

  void start() {
    // Start listening on the '/car/camera' topic
    ROS_INFO("Started listening to the /car/camera topic");
    subscriber = nodeHandle.subscribe("/car/camera", 1000, &TrackFinder::didReceiveCones, this);
  }

  void didReceiveCones(const track::Cones::ConstPtr& cones) {
    ROS_INFO("New information received on the car/camera topic");
    // Calculate the centerline
    track::Line centerLine = findCenterLine(cones->cones);

    // Publish the centerline
    publisher.publish(centerLine);
  }


  track::Line findCenterLine(std::vector<track::Cone> visibleCones) {
   
    // ACTUAL INPUT
    int n_cones = visibleCones.size();

    // Initialize labels and coordinates
    int cones_col[n_cones];
    float cones_pos[n_cones][2];
    float cones_x[n_cones];
    float cones_y[n_cones];

    // Fill labels and coordinates with data 
    for (int i=0; i<n_cones; i++)
    {
      cones_col[i] = visibleCones[i].color;
      cones_pos[i][0] = visibleCones[i].position.x;
      cones_pos[i][1] = visibleCones[i].position.y;
      cones_x[i] = visibleCones[i].position.x;
      cones_y[i] = visibleCones[i].position.y;
    };


    // Set up training data
    Mat labelsMat(n_cones, 1, CV_32S, cones_col);
    Mat trainingDataMat(n_cones, 2, CV_32FC1, cones_pos);
    
    // Set up SVM's parameters
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::POLY);
    svm->setDegree(2.0);
    // svm->setNu(0.1);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));

    // Train the SVM with given parameters
    Ptr<TrainData> td = TrainData::create(trainingDataMat, ROW_SAMPLE, labelsMat);
    svm->train(td);

    // Or train the SVM with optimal parameters
    // svm->trainAuto(td);

    // Data for the map
  
    int x_min = *min_element(cones_x, cones_x + n_cones);
    int x_max = *max_element(cones_x, cones_x + n_cones);
    int y_min = *min_element(cones_y, cones_y + n_cones);
    int y_max = *max_element(cones_y, cones_y + n_cones);

    Mat map = Mat::zeros(600, 600, CV_8UC3);
    ROS_INFO_STREAM("Creating a map. x: {" << x_min << "," << x_max << "}");

    // Initialize vector to store points on centerline
    std::vector<track::Point> centerLinePoints;

    Vec3b blue(240, 150, 100), yellow(100, 255, 255);
    // Show the decision regions given by the SVM
    for (int i = 1; i < map.rows; ++i)
        for (int j = 1; j < map.cols; ++j)
        {
            Mat prev_sampleMat = (Mat_<float>(1,2) << j-1, i-1);
            Mat sampleMat = (Mat_<float>(1, 2) << j, i);
            float prev_response = svm->predict(prev_sampleMat);
            float response = svm->predict(sampleMat);

            if (response != prev_response){
              track::Point pt;
              pt.x = j;
              pt.y = i;
              centerLinePoints.push_back(pt);

              circle(map, Point(j,i), 2, Scalar(0,0,0), -1, 8);
            }

            // For decorative purposes
            if (response == 1){
              map.at<Vec3b>(i,j) = yellow;
            }
            else if (response == -1){
              map.at<Vec3b>(i,j) = blue;
            }
                
        }

    // Show the training data
    for (int i = 0; i < n_cones; ++i)
    {
      float x = cones_x[i];
      float y = cones_y[i];
      if (cones_col[i] == 1){
        drawMarker(map,Point(x,y),Scalar(10,180,255),5,15,2,8);   // Yellow Cones
      } else {
        drawMarker(map,Point(x,y),Scalar(255,0,0),5,15,2,8);      // Blue Cones
      }
     
    }

    // Show support vectors
    Mat sv = svm->getSupportVectors();

    for (int i = 0; i < sv.rows; ++i)
    {
        const float* v = sv.ptr<float>(i);
        circle(map, Point((int)v[0], (int)v[1]), 8, Scalar(0,0,255), 2, 8);   // Red circles indicate Support Vectors
    }

    imshow("SVM Simple Example", map); // show it to the users
    waitKey(0);
    



    /*
    //------------------------------------------------//
    Alex's altnernative classification loop
    //----------------------------------------------//
    // Create an empty "map"  
    int x_max = *std::max_element(cones_x, cones_x+n_cones) - *std::min_element(cones_x, cones_x+n_cones);     
    int y_max = *std::max_element(cones_y, cones_y+n_cones);
    cv::Mat map = cv::Mat::zeros(y_max,x_max,CV_8UC3);

    int initial_guess, final_guess;
    int x_coord, y_coord;
    int prediction, prev_prediction;
    */
    /*
    // Fill matrix with classification
    for (int i = 1; i < y_max; i++)     // loop through rows, omit first entry
    {
      for (int j = 0; j < x_max; j++)   // loop through columns, omit first entry
        // {
        //     if (i == 1){
        //         cv::Mat firstSampleMate = (cv::Mat_<float>(1,2) << j-1, i-1);
        //         float prev_response = svm->predict(firstSampleMate);
        //     } else{
        //         Mat sampleMat = (Mat_<float>(1,2) << j,i );
        //         float prev_response = response;
        //         float response = svm->predict(sampleMat);
        //     }
        //     if (response == prev_response){
        //         continue;
        //     } else if (response != prev_response){
        //         track::Point pt;
        //         pt.x = i;
        //         pt.y = j;
        //         centerLine.push_back(pt);
        //     }
        {
          if (i == 1){
            initial_guess = 0;
            x_coord = 0;
            cv::Mat prev_predict_coord = (cv::Mat_<float>(1,2) << x_coord, y_coord-1);
            prev_prediction = svm->predict(prev_predict_coord);

        if (i == 1){
          initial_guess = 0;
          x_coord = 0;
          cv::Mat predict_coord = (cv::Mat_<float>(1,2) << x_coord, y_coord-1);
          prev_prediction = svm->predict(predict_coord);

          cv::Mat predict_coord2 = (cv::Mat_<float>(1,2) << x_coord, y_coord);
          prediction = svm->predict(predict_coord2);

        } else {
          initial_guess = final_guess;
          x_coord = -(abs(x_coord)+1);     //Start in the center, and move to the sides
          cv::Mat predict_coord = (cv::Mat_<float>(1,2) << x_coord-1, y_coord-1);
          prediction = svm->predict(predict_coord);
        }
        int y_coord = i;

        if (prediction == prev_prediction){
          continue;
        } else {
          final_guess = x_coord;
          track::Point pt;
          pt.x = x_coord;
          pt.y = y_coord;
          centerLine.push_back(pt);
        }
      }
    }
    */

    

    track::Line centerLine;
    centerLine.points = centerLinePoints;
    return centerLine;
    }
};




int main(int argc, char **argv) {

  // Init the node
  ROS_INFO("Starting 'trackfinder' node...");
  ros::init(argc, argv, "trackfinder");

  // Create a TrackFinder object
  TrackFinder trackFinder;

  // Start the feedback loop
  trackFinder.start();

  // Directly run findCenterLine()
  //vector<track::Cone> cones;
  //trackFinder.findCenterLine(cones);

  // Keep listening till Ctrl+C is pressed
  ros::spin();

  // Exit succesfully
  return 0;
}