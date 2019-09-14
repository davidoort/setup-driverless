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
    subscriber = nodeHandle.subscribe("/car/camera", 1000, &TrackFinder::didReceiveCones, this);
  }

  void didReceiveCones(const track::Cones::ConstPtr& cones) {

    // Calculate the centerline
    track::Line centerLine = findCenterLine(cones->cones);

    // Publish the centerline
    publisher.publish(centerLine);
  }

  track::Line findCenterLine(vector<track::Cone> visibleCones) {

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