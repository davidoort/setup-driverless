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
  // create client needed to request from ben
  ros::ServiceClient client = n.serviceClient<track::Generator>("/track/generate");
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

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("cones_visualization", 10); 
  ros::Rate r(30);

  // Extract a centerline from the track message
  track::Line centerline = srv.response.track.centreline;

  // Extract cones (to be published later) from the track message
  track::Cones cones = srv.response.track.cones;

  while (ros::ok())
  {

	  visualization_msgs::Marker points;
	  points.header.frame_id = "/cones";
	  points.header.stamp = ros::Time::now();
	  points.ns = "track_cones_location";
	  points.action = visualization_msgs::Marker::ADD;
	  points.id = 0;
	  points.type = visualization_msgs::Marker::POINTS;
	  // points.type = visualization_msgs::Marker::MESH_RESOURCE;
      // points.mesh_resource = "package://simulation/meshes/cone_blue.dae";

	  points.scale.x = 0.2;
	  points.scale.y = 0.2;
	  points.color.a = 1.0;

	 
	  int z = 0;

	  points.lifetime = ros::Duration();

	  // Wait untill there is at least one subsriber to publish
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber. Run rviz.");
        sleep(1);
      }


	  int size = srv.response.track.cones.cones.size();
	  for (int i=0; i<size; i++){
	    // ROS_INFO("%f, %f, %i", cones.cones[i].position.x, cones.cones[i].position.y, cones.cones[i].color);

	    geometry_msgs::Point p;
	    std_msgs::ColorRGBA pc;
	    p.x = cones.cones[i].position.x;
	    p.y = cones.cones[i].position.y;
	    p.z = z;

	    if (cones.cones[i].color == false){
	    	pc.r =(float) 0/255; pc.g =(float) 128/255; pc.b =(float) 255/255; pc.a=1;	  		
	    }
	    else{
	    	pc.r =(float) 255/255; pc.g =(float) 255/255; pc.b =(float) 0/255; pc.a=0.5;
	    }

	    points.points.push_back(p);
	    points.colors.push_back(pc);
	  }

	  marker_pub.publish(points);
	  ROS_INFO("Visualization generated.");

	  r.sleep();


	  return 0;

	  //ros::Subscriber sub = n.subscribe("/car/camera", 1000, show_visible_cones); // 



  }
}