#include "ros/ros.h"
#include "track/Generator.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"
#include <array>
#include <visualization_msgs/Marker.h>
#include <cmath>

/*void show_visible_cones()
{
	ROS_INFO("Message detected");
}*/

int main(int argc, char **argv)
{
  // Initialize ROS; name of the node: "visualize_track"
  ros::init(argc, argv, "visualize_track");
  ros::NodeHandle n;

  // ros::ServiceClient client = n.serviceClient<PACKAGE_NAME::SERVICE_NAME>("SERVICE_NAME");
  // create client needed to request from generator
  ros::ServiceClient client = n.serviceClient<track::Generator>("/track/generate");
  ros::Publisher track_pub = n.advertise<visualization_msgs::Marker>("track_visualization", 100); 
  ros::Publisher centreline_pub = n.advertise<visualization_msgs::Marker>("shape_visualization", 10);
  //ros::Publisher car_pub = n.advertise<visualization_msgs::Marker>("car_visualization", 100);

  ros::Rate r(30);

  ROS_INFO("Ready to call generator service.");
  track::Generator srv;

  if (client.call(srv))
  {
    ROS_INFO("Requested cones from track generator.");
  }
  else
  {
    ROS_ERROR("Failed to call service from generator.");
  }

  // Extract a centerline from the track message
  track::Line centerline = srv.response.track.centreline;

  // Extract cones (to be published later) from the track message
  track::Cones cones = srv.response.track.cones;

  while (ros::ok())
  {

	  visualization_msgs::Marker points, line_strip;
	  points.header.frame_id = line_strip.header.frame_id = "/cones";
	  points.header.stamp = line_strip.header.stamp = ros::Time::now();
	  points.ns = line_strip.ns = "track_cones_location";
	  points.action = line_strip.action = visualization_msgs::Marker::ADD;
	  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

	  points.id = 0;
	  line_strip.id = 1;

	  points.type = visualization_msgs::Marker::POINTS;
	  // points.type = visualization_msgs::Marker::MESH_RESOURCE;
      // points.mesh_resource = "package://simulation/meshes/cone_blue.dae";
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;

	  points.scale.x = 0.2;
	  points.scale.y = 0.2;
	  points.color.a = 1.0;
	  line_strip.scale.x = 0.1;

	  // Line strip is blue
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;

	 
	  int z = 0;

	  points.lifetime = ros::Duration();

	  // Wait untill there is at least one subsriber to publish
      while (track_pub.getNumSubscribers() < 1 && centreline_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber. Run rviz.");
        sleep(1);
      }


	  int size = srv.response.track.cones.cones.size();
	  int centerline_size = srv.response.track.centreline.points.size();
	  for (int i=0; i<size; i++){
	    // ROS_INFO("%f, %f, %i", cones.cones[i].position.x, cones.cones[i].position.y, cones.cones[i].color);

	    geometry_msgs::Point p;
	    std_msgs::ColorRGBA pc;
	    p.x = cones.cones[i].position.x;
	    p.y = cones.cones[i].position.y;
	    p.z = z;

	    if (cones.cones[i].color == 0){
	    	pc.r =(float) 0/255; pc.g =(float) 128/255; pc.b =(float) 255/255; pc.a=1;	  		
	    }
	    else{
	    	pc.r =(float) 255/255; pc.g =(float) 255/255; pc.b =(float) 0/255; pc.a=0.5;
	    }

	    points.points.push_back(p);
	    points.colors.push_back(pc);
	  }

	  for (int k=0; k<centerline_size; k++){
		  geometry_msgs::Point c;
		  c.x = centerline.points[k].x;
		  c.y = centerline.points[k].y;
		  c.z = z;
		  line_strip.points.push_back(c);
	  }

	  // line_strip.points = centerline.points;

	  track_pub.publish(points);
	  track_pub.publish(line_strip);
	  ROS_INFO("Visualization generated.");

	  r.sleep();
	  sleep(2);


	  // Publish another entity.


	  uint32_t shape = visualization_msgs::Marker::SPHERE;

	  visualization_msgs::Marker marker;
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "/cones";
	    marker.header.stamp = ros::Time::now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    marker.ns = "cone_shapes";
	    marker.id = 0;

	    // Set the marker type. 
	    marker.type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = 0;
	    marker.pose.position.y = 0;
	    marker.pose.position.z = 0.4;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 1.0;
	    marker.scale.y = 1.0;
	    marker.scale.z = 1.0;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 0.5;

	    marker.lifetime = ros::Duration();

	    centreline_pub.publish(marker);

	    sleep(3);

	  return 0;

	  //ros::Subscriber sub = n.subscribe("/car/camera", 1000, show_visible_cones); // 



  }
}