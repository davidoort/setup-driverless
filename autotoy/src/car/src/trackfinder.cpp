#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include "ros/ros.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"


class TrackMap 
{
  public:
  std::vector<track::Cone> seen_cones;
  std::vector<track::Cone> seen_cones_color;
  std::vector<track::Cone> seen_cones_coords;

  bool handle_new_cone(track::Cones new_cones)
  {
    for (int i=0; i<new_cones.cones.size(); i++)
    {
      seen_cones.push_back(new_cones.cones[i]);
      seen_cones_color.push_back(new_cones.cones[i].color);
      seen_cones_coords.push_back({new_cones.cones[i].position.x, new_cones.cones[i].position.y});
    }

    // Set up training data
    int labels[] = seen_cones_color;
    float cones_coords[][] = seen_cones_coords;
    cv::Mat trainingDataMat(cones_coords.size(), cones_coords[0].size(), CV_32F, cones_coords);
    cv::Mat labelsMat(labels.size(), 1, CV_32SC1, labels);

    // Train the SVM
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    svm->cv::ml::setType(cv::ml::SVM::C_SVC);
    svm->cv::ml::setKernel(cv::ml::SVM::LINEAR);
    svm->cv::ml::setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));
    svm->cv::ml::train(trainingDataMat, cv::ml::ROW_SAMPLE, labelsMat);

  return true;
  }

  plan_track()
  {
        // Show the decision regions given by the SVM
    Vec3b green(0,255,0), blue(255,0,0);
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            Mat sampleMat = (Mat_<float>(1,2) << j,i);
            float response = svm->predict(sampleMat);
            if (response == 1)
                image.at<Vec3b>(i,j)  = green;
            else if (response == 0)
                image.at<Vec3b>(i,j)  = blue;
        }
    }

  }

};


int main(int argc, char **argv) {

  TrackMap track_map;

  // Init node with name 'trackfinder'
  ros::init(argc, argv, "trackfinder");

  ros::NodeHandle n;
  ros::Publisher send_track = n.advertise<track::Point>("/car/targetline", 1000);
  ros::Subscriber camera_input = n.subscribe("/track/camera", 1000, track_map.handle_new_cone);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {


    ros::spinOnce();


    track::Line msg;

    send_track.publish(msg);


    loop_rate.sleep();
  }


  return 0;
}