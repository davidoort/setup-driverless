#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

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
    subscriber = nodeHandle.subscribe("/car/camera", 1000, &TrackFinder::didReceiveCones, this);
  }

  void didReceiveCones(const track::Cones::ConstPtr& cones) {

    // Calculate the centerline
    track::Line centerLine = findCenterLine(cones->cones);

    // Publish the centerline
    publisher.publish(centerLine);
  }

  track::Line findCenterLine(std::vector<track::Cone> visibleCones) {

    ROS_INFO("Finding center line...");

    // Data for visual representation
    int width = 512, height = 512;
    Mat image = Mat::zeros(height, width, CV_8UC3);

    // Set up training data
    int labels[6] = { 1, 1, 1, -1, -1, -1 };

    Mat labelsMat(6, 1, CV_32S, labels);

    float trainingData[6][2] = { { 100, 150 }, { 100, 450 }, { 200, 300 }, { 300, 150 }, { 300, 450 }, { 400, 300} };
    Mat trainingDataMat(6, 2, CV_32FC1, trainingData);

    // Set up SVM's parameters
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::POLY);
    svm->setDegree(2.0);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));

    // Train the SVM with given parameters
    Ptr<TrainData> td = TrainData::create(trainingDataMat, ROW_SAMPLE, labelsMat);
    svm->train(td);

    // Or train the SVM with optimal parameters
    //svm->trainAuto(td);

    Vec3b green(0, 255, 0), blue(255, 0, 0);
    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            Mat sampleMat = (Mat_<float>(1, 2) << j, i);
            float response = svm->predict(sampleMat);

            if (response == 1)
                image.at<Vec3b>(i, j) = green;
            else if (response == -1)
                image.at<Vec3b>(i, j) = blue;
        }

    // Show the training data
    int thickness = -1;
    int lineType = 8;
    //circle(image, Point(501, 10), 5, Scalar(0, 0, 0), thickness, lineType);
    //circle(image, Point(255, 10), 5, Scalar(255, 255, 255), thickness, lineType);
    //circle(image, Point(501, 255), 5, Scalar(255, 255, 255), thickness, lineType);
    //circle(image, Point(10, 501), 5, Scalar(255, 255, 255), thickness, lineType);

    // Show support vectors
    thickness = -1;
    lineType = 8;
    Mat sv = svm->getSupportVectors();

    for (int i = 0; i < sv.rows; ++i)
    {

        ROS_INFO("Drawing circle");
        const float* v = sv.ptr<float>(i);
        circle(image, Point((int)v[0], (int)v[1]), 6, Scalar(255, 255, 255), thickness, lineType);
    }

    ROS_INFO("Stored center line in result.png!");

    imwrite("/var/tmp/opencv-img.png", image);        // save the image

    imshow("SVM Simple Example", image); // show it to the user
    waitKey(0);




    track::Line centerLine;

    // Put one demo point in the center line
    track::Point demoPoint;
    demoPoint.x = 1.0;
    demoPoint.y = 2.0;
    std::vector<track::Point> centerLinePoints;
    centerLinePoints.push_back(demoPoint);
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
  //trackFinder.start();

  // Directly run findCenterLine()
  vector<track::Cone> cones;
  trackFinder.findCenterLine(cones);

  // Keep listening till Ctrl+C is pressed
  //ros::spin();

  // Exit succesfully
  return 0;
}