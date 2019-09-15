#include <ros/ros.h>
#include <track/Generator.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "conemap");
  ros::NodeHandle n;
  // ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // ros::ServiceClient client = n.serviceClient<PACKAGE_NAME::SERVICE_NAME>("SERVICE_NAME");
  // create client needed to request from ben
  ros::ServiceClient client = n.serviceClient<track::Generator>("/track/generate_track"); 

  ros::Rate r(30);

  ROS_INFO("Ready to call Ben.");

  track::Generator srv;

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = "/cones";
    points.header.stamp = ros::Time::now();
    points.ns  = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;



    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;


    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;


    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);

    marker_pub.publish(points);

    r.sleep();

    //f += 0.04;
  }
}
}

// Fusion this with Bencall.cpp !!