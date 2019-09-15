#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "cone_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("cones_visualization", 1);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/cones_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cone_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://map/meshes/cone_blue.dae";
    marker.action = visualization_msgs::Marker::ADD;

    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    // marker.color.r = 0.0f;
    // marker.color.g = 1.0f;
     marker.color.b = 0.9f;
     marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    

    for (uint i=0; i<10; ++i)
    {
    visualization_msgs::Marker p;
    p.pose.position.x = i;
    p.pose.position.y = 0;
    p.pose.position.z = 0.15;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;



    	// marker.push_back(p);
    }



    marker_pub.publish(marker);

    r.sleep();
  }
}

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 
