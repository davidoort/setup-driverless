#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "cone_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("cones_visualization", 100);

  while (ros::ok())
  {
    /*visualization_msgs::Marker marker;
    marker.header.frame_id = "/cones";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cone_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://simulation/meshes/cone_blue.dae";
    marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      // Set the color -- be sure to set alpha to something non-zero!
       marker.color.g = 0.9f;
       marker.color.a = 1.0;

    
    marker.lifetime = ros::Duration();*/

    // Publish the marker
    /*while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }*/
    
    //std::vector<visualization_msgs::Marker> newcones;
    visualization_msgs::MarkerArray amarker;
    
    for (uint i=0; i<10; ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/cones";
      marker.header.stamp = ros::Time::now();
      marker.ns = "cone_shapes";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://simulation/meshes/cone_blue.dae";
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      marker.pose.position.x = i;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0.15;

      marker.pose.orientation.w = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 1;
      marker.pose.orientation.z = 1;

      // Set the color -- be sure to set alpha to something non-zero!
       marker.color.g = 0.9f;
       marker.color.a = 1.0;

    
    marker.lifetime = ros::Duration();
    amarker.markers.push_back(marker);
    

    //sleep(3);
     
    }
    //ROS_INFO()
    marker_pub.publish(amarker);
    //marker_pub.publish(marker);

    //marker_pub.publish(newcones);

    r.sleep();
  }
}

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 
