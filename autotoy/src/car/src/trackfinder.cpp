#include "ros/ros.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"
#include <math.h>

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
   // stringstream ss;
   // ss << "Received " << cones->cones.size() << " new cones!";
   // string msg = ss.str();
    ROS_INFO("Received new cones!");

    // Calculate the centerline
    track::Line centerLine = findCenterLine(cones->cones);

    // Publish the centerline
    publisher.publish(centerLine);
  }

  float getMiddle(float val1, float val2) {
    if (val1 < val2) { return val1 + (val2-val1)/2.f; }
    return val2 + (val1-val2)/2.f;
  }

  track::Line findCenterLine(std::vector<track::Cone> visibleCones) {
    stringstream ss;
    ss << "Calculation centerline for " << visibleCones.size() << " cones: [";
    for (int i = 0; i < visibleCones.size(); i++) {
      ss << "(" << visibleCones[i].position.x << ", " << visibleCones[i].position.y << ", " << visibleCones[i].color << "), ";
    }
    ss << "]";
    // ROS_INFO_STREAM(ss.str());

    float leftX = numeric_limits<float>::max();
    track::Point lowestLeftCone;
    lowestLeftCone.y = numeric_limits<float>::max();
    track::Point lowestRightCone;
    lowestRightCone.y = numeric_limits<float>::max();

    vector<track::Cone> leftCones;
    vector<track::Cone> rightCones;

    for (int i = 0; i < visibleCones.size(); i++) {
      if (visibleCones[i].color == 0) { 
        leftCones.push_back(visibleCones[i]);
        if (visibleCones[i].position.y < lowestLeftCone.y) { lowestLeftCone = visibleCones[i].position; }
      }
      if (visibleCones[i].color == 1) {
        rightCones.push_back(visibleCones[i]);
        if (visibleCones[i].position.y < lowestRightCone.y) { lowestRightCone = visibleCones[i].position; }
      }
    }

    float cameraX = getMiddle(lowestLeftCone.x, lowestRightCone.x);
    float cameraY = getMiddle(lowestLeftCone.y, lowestRightCone.y);

    track::Line centerLine;

    while(leftCones.size() > 0 && rightCones.size() > 0) {
      float shortestDistance = numeric_limits<float>::max();
      int nearestLeftConeIndex;
      for (int i = 0; i < leftCones.size(); i++) {
        float distance = sqrt(pow((leftCones[i].position.x - cameraX), 2.0) + pow((leftCones[i].position.y - cameraY), 2.0));
        if (distance < shortestDistance) {
          shortestDistance = distance;
          nearestLeftConeIndex = i;
        }
      }

      shortestDistance = numeric_limits<float>::max();
      int nearestRightConeIndex;
      for (int i = 0; i < rightCones.size(); i++) {
        float distance = sqrt(pow((rightCones[i].position.x - cameraX), 2.0) + pow((rightCones[i].position.y - cameraY), 2.0));
        if (distance < shortestDistance) {
          shortestDistance = distance;
          nearestRightConeIndex = i;
        }
      }

      track::Point centerPoint;
      centerPoint.x = getMiddle(leftCones[nearestLeftConeIndex].position.x, rightCones[nearestRightConeIndex].position.x);
      centerPoint.y = getMiddle(leftCones[nearestLeftConeIndex].position.y, rightCones[nearestRightConeIndex].position.y);
      centerLine.points.push_back(centerPoint);

      leftCones.erase(leftCones.begin() + nearestLeftConeIndex);
      rightCones.erase(rightCones.begin() + nearestRightConeIndex);
    }

    stringstream ss2;
    ss2 << "Sending " << centerLine.points.size() << " centerline points: [";
    for (int i = 0; i < centerLine.points.size(); i++) {
      ss2 << "(" << centerLine.points[i].x << ", " << centerLine.points[i].y << "), ";
    }
    ss2 << "]";
    // ROS_INFO_STREAM(ss2.str());

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

  // Keep listening till Ctrl+C is pressed
  ros::spin();

  // Exit succesfully
  return 0;
}