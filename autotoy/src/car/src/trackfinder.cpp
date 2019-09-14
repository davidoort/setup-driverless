#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
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
    subscriber = nodeHandle.subscribe<track::Cones>("/car/camera", 1000, didReceiveCones);
  }

  void didReceiveCones(const track::Cones::ConstPtr& cones) {

    // Calculate the centerline
    track::Line centerLine = findCenterLine(cones->cones);

    // Publish the centerline
    publisher.publish(centerLine);
  }

  track::Line findCenterLine(vector<track::Cone> visibleCones) {

    // for (int i = 0; i < new_cones.cones.size; i++) {
    //   seen_cones.push_back(new_cones.cones[i]);
    //   seen_cones_color.push_back(new_cones.cones[i].color);
    //   seen_cones_coords.push_back(new_cones.cones[i].position);
    // }

    // // Set up training data
    // int labels[] = seen_cones_color;
    // float cones_coords[][] = seen_cones_coords;
    // cv::Mat trainingDataMat(cones_coords.size(), cones_coords[0].size(), CV_32F, cones_coords);
    // cv::Mat labelsMat(labels.size(), 1, CV_32SC1, labels);

    // // Train the SVM
    // cv::Ptr<SVM> svm = cv::ml::SVM::create();
    // svm->cv::ml::setType(cv::ml::SVM::C_SVC);
    // svm->cv::ml::setKernel(cv::ml::SVM::LINEAR);
    // svm->cv::ml::setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));
    // svm->cv::ml::train(trainingDataMat, cv::ml::ROW_SAMPLE, labelsMat);

    // // Show the decision regions given by the SVM
    // Vec3b green(0,255,0), blue(255,0,0);
    // for (int i = 0; i < image.rows; i++)
    // {
    //     for (int j = 0; j < image.cols; j++)
    //     {
    //         Mat sampleMat = (Mat_<float>(1,2) << j,i);
    //         float response = svm->predict(sampleMat);
    //         if (response == 1)
    //             image.at<Vec3b>(i,j)  = green;
    //         else if (response == 0)
    //             image.at<Vec3b>(i,j)  = blue;
    //     }
    // }


    track::Line centerLine;

    // Put one demo point in the center line
    track::Point demoPoint;
    demoPoint.x = 1.0;
    demoPoint.y = 2.0;
    vector<track::Point> centerLinePoints;
    centerLinePoints.push_back(demoPoint);
    centerLine.points = centerLinePoints;

    return centerLine;
  }
};

int main(int argc, char **argv) {

  // Init the node
  ROS_DEBUG("Starting 'trackfinder' node...");
  ros::init(argc, argv, "trackfinder");

  // Create a TrackFinder object
  TrackFinder trackFinder;

  // Start the feedback loop
  trackFinder.start();

  // Keep listening till Ctrl+C is pressed
  ros::spin();

  // Exit succesfully
  return 0;
}