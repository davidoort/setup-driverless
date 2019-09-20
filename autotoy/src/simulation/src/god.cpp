#include "ros/ros.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Generator.h"
#include "track/Line.h"
#include "car/Location.h"
#include "car/Control.h"
#include "car/Velocity.h"
#include <tuple>
#include <cmath>
#include <array>
#include <visualization_msgs/Marker.h>

/* 
This script is supposed to define the functionality of the "GodNode" which is the simulator. 
- First of all, it starts the simulation by defining a client of the "/track/generate" service which returns a track. 
- It then has to publish the location of the car to the /car/location topic and the location (and color) of the cones to the /track/cones topic
- It will subscribe to /car/location topic to receive control command messages, which will be passed through a car model and the car location will be updated.
- It will subscribe to a bunch of extra topics such as car/camera, car/targetline to be able to represent these things in RViz and help with debugging
*/

// in seconds
#define timeBetweenTick 0.1

class Car {
public:

  float x; // meter
  float y; // meter
  float heading; // radians
  float velocity; // meter/second
  float acceleration; // meter/second^2
  float yawrate; // radians/second

  Car() : x(0.0), y(0.0), heading(0.0), velocity(0.0), acceleration(0.0), yawrate(0.0) {}

  void move() {
    velocity = velocity + acceleration*timeBetweenTick;
    heading = heading + yawrate*timeBetweenTick;
    x = x + velocity*timeBetweenTick*cos(heading);
    y = y + velocity*timeBetweenTick*sin(heading);
  }

  void control(const float accel, const float newYawrate) {
    yawrate = newYawrate;
    acceleration = accel;
  }
};

class Simulator {
public:
  ros::Publisher cones_pub, car_location_pub, track_pub, centreline_pub, car_vis_pub, cone_vis_pub, target_pub, yaw_vis_pub, car_velocity_pub;
  ros::Subscriber car_cmd_sub, visible_cones_sub, target_path_sub;
  ros::NodeHandle n;
  Car* car;



  std::tuple<track::Line, track::Cones> getTrack(ros::NodeHandle& n) {
    // ros::ServiceClient client = n.serviceClient<PACKAGE_NAME::SERVICE_NAME>("SERVICE_NAME");
    // create client needed to request from generator
    ros::ServiceClient client = n.serviceClient<track::Generator>("/track/generate");
    ROS_INFO("Ready to call generator service.");
    track::Generator srv;

    if (client.call(srv)){
      ROS_INFO("Received track from track generator.");

      // Extract a centerline from the track message
      track::Line centerline = srv.response.track.centreline;

      // Extract cones (to be published later) from the track message
      track::Cones cones = srv.response.track.cones;
      return std::make_tuple(centerline, cones);
    }
    ROS_ERROR("Failed to call service /track/generate");
    ros::shutdown(); //  this shuts down all publishers (and everything else).
    return std::make_tuple(track::Line(), track::Cones());
  }

  int publish_Cones(ros::NodeHandle& n, track::Cones& cones, track::Line& centreline) {
    ROS_INFO("Publishing cones!");
    cones_pub = n.advertise<track::Cones>("/track/cones", 1, true);
    track_pub = n.advertise<visualization_msgs::Marker>("track_visualization", 100); 
    centreline_pub = n.advertise<visualization_msgs::Marker>("shape_visualization", 10);
    cones_pub.publish(cones);

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
  points.color.a = 0.5;
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


  int size = cones.cones.size();
  int centerline_size = centreline.points.size();
  for (int i=0; i<size; i++){
  //ROS_INFO("%f, %f, %i", cones.cones[i].position.x, cones.cones[i].position.y, cones.cones[i].color);

  geometry_msgs::Point p;
  std_msgs::ColorRGBA pc;
  p.x = cones.cones[i].position.x;
  p.y = cones.cones[i].position.y;
  p.z = z;

  if (cones.cones[i].color == 0){
    pc.r =(float) 0/255; pc.g =(float) 200/255; pc.b =(float) 255/255; pc.a=0.5;        
  }
  else{
    pc.r =(float) 255/255; pc.g =(float) 255/255; pc.b =(float) 192/255; pc.a=0.5;
  }

  points.points.push_back(p);
  points.colors.push_back(pc);
  }

  for (int k=0; k<centerline_size; k++){
    geometry_msgs::Point c;
    c.x = centreline.points[k].x;
    c.y = centreline.points[k].y;
    c.z = z;
    line_strip.points.push_back(c);
  }

  // line_strip.points = centerline.points;

  track_pub.publish(points);
  track_pub.publish(line_strip);
  ROS_INFO("Visualization generated.");
  return 1;

  //r.sleep();
  sleep(2);

  }

  void publish_car(ros::NodeHandle& n, car::Location& location)
  {
    
    car_vis_pub =  n.advertise<visualization_msgs::Marker>("car_visualization", 100);
    yaw_vis_pub =  n.advertise<visualization_msgs::Marker>("yaw_visualization", 100);
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker, yaw_arrow;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = yaw_arrow.header.frame_id = "/cones";
    marker.header.stamp = yaw_arrow.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = yaw_arrow.ns = "car_location";
    marker.id = yaw_arrow.id = 2;

    // Set the marker type. 
    marker.type = shape;
    yaw_arrow.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = yaw_arrow.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = yaw_arrow.pose.position.x = location.location.x;
    marker.pose.position.y = yaw_arrow.pose.position.y = location.location.y;
    marker.pose.position.z = yaw_arrow.pose.position.z = 0.4;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    // yaw (Z), pitch (Y), roll (X)
    double yaw = location.heading;
    double pitch = 0.0;
    double roll = 0.0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    yaw_arrow.pose.orientation.x = cy * cp * sr - sy * sp * cr;
    yaw_arrow.pose.orientation.y = sy * cp * sr + cy * sp * cr;
    yaw_arrow.pose.orientation.z = sy * cp * cr - cy * sp * sr;
    yaw_arrow.pose.orientation.w = cy * cp * cr + sy * sp * sr;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x  =1.0;
    marker.scale.y  =1.0;
    marker.scale.z  =1.0;
    yaw_arrow.scale.x =2.0;
    yaw_arrow.scale.y =0.2;
    yaw_arrow.scale.z =0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    yaw_arrow.color.r = 1.0f;
    yaw_arrow.color.a = 1.0;

    marker.lifetime = yaw_arrow.lifetime = ros::Duration();

    car_vis_pub.publish(marker);
    yaw_vis_pub.publish(yaw_arrow);
    ROS_INFO("Updated car location!");

    //sleep(3); 
  }

  void controlCommandReceived(const car::Control& control)
  {
    car->control(control.acceleration, control.yawrate);
    ROS_INFO("Received control command!");
  }

  void visableConesReceived(const track::Cones& visible_cones) {
    ROS_INFO("Received visible cones!");
    cone_vis_pub = n.advertise<visualization_msgs::Marker>("visible_cones", 100);

    visualization_msgs::Marker cones;
    cones.header.frame_id  = "/cones";
    cones.header.stamp = ros::Time::now();
    cones.ns = "visible_cones_location";
    cones.action =  visualization_msgs::Marker::ADD;
    cones.pose.orientation.w = 1.0;

    cones.id = 4;
    

    cones.type = visualization_msgs::Marker::POINTS;
    // points.type = visualization_msgs::Marker::MESH_RESOURCE;
    // points.mesh_resource = "package://simulation/meshes/cone_blue.dae";

    cones.scale.x = 0.4;
    cones.scale.y = 0.4;
    cones.color.a = 1.0;

    int z = 0;

    cones.lifetime = ros::Duration();
    int size = visible_cones.cones.size();
    for (int i=0; i<size; i++){
    //ROS_INFO("%f, %f, %i", cones.cones[i].position.x, cones.cones[i].position.y, cones.cones[i].color);

    geometry_msgs::Point p;
    std_msgs::ColorRGBA pc;
    p.x = visible_cones.cones[i].position.x;
    p.y = visible_cones.cones[i].position.y;
    p.z = z;

    if (visible_cones.cones[i].color == 0){
      pc.r =(float) 0/255; pc.g =(float) 200/255; pc.b =(float) 255/255; pc.a=1;        
    }
    else{
      pc.r =(float) 255/255; pc.g =(float) 255/255; pc.b =(float) 0/255; pc.a=1;
    }

    cones.points.push_back(p);
    cones.colors.push_back(pc);
    }


    cone_vis_pub.publish(cones);
    ROS_INFO("Publishing visible cones!");

  }

  void targetPathReceived(const track::Line& target_path) {
    // Show these cones in RViz
    ROS_INFO("Received target path!");
    target_pub = n.advertise<visualization_msgs::Marker>("target_visualization", 100); 

    visualization_msgs::Marker target_line;
    target_line.header.frame_id = "/cones";
    target_line.header.stamp = ros::Time::now();
    target_line.ns = "track_cones_location";
    target_line.action = visualization_msgs::Marker::ADD;
    target_line.pose.orientation.w = 1.0;
    target_line.id = 5;
    target_line.type = visualization_msgs::Marker::LINE_STRIP;
    target_line.scale.x = 0.1;
    target_line.color.b = 1.0;
    target_line.color.a = 1.0;


    int z = 0;

    target_line.lifetime = ros::Duration();
    int targetline_size = target_path.points.size();
    
    for (int t=0; t<targetline_size; t++){
      geometry_msgs::Point c;
      c.x = target_path.points[t].x;
      c.y = target_path.points[t].y;
      c.z = z;
      target_line.points.push_back(c);
    }

    // line_strip.points = target_path.points;
    target_pub.publish(target_line);
    ROS_INFO("Target path generated.");
  }

  void setCar(Car* newCar) {
    car = newCar;
  }
};







int main(int argc, char **argv)
{
  // Initialize ROS; name of the node: "visualize_track"
  ros::init(argc, argv, "visualize_track");

  Car car;
  Simulator simulator;
  simulator.setCar(&car);

  track::Line centerline;
  track::Cones cones;

  tie(centerline, cones) = simulator.getTrack(simulator.n);

  simulator.publish_Cones(simulator.n, cones, centerline);

  //ros::Publisher car_pub = n.advertise<visualization_msgs::Marker>("car_visualization", 100);

  // Initialize publishers and subscribers
  simulator.car_velocity_pub = simulator.n.advertise<car::Velocity>("car/velocity", 100);
  simulator.car_location_pub = simulator.n.advertise<car::Location>("/car/location", 100);

  simulator.car_cmd_sub = simulator.n.subscribe("/car/controls", 100, &Simulator::controlCommandReceived, &simulator); // Not sure if the first & is needed
  simulator.visible_cones_sub = simulator.n.subscribe("/car/camera", 100, &Simulator::visableConesReceived, &simulator);
  simulator.target_path_sub = simulator.n.subscribe("/car/targetline", 100, &Simulator::targetPathReceived, &simulator);

  ros::Rate loop_rate(1/timeBetweenTick);


  while (ros::ok())
  {
    car.move();
    
    car::Location location;
    car::Velocity velocity;
    
    location.location.x = car.x;
    location.location.y = car.y;
    location.heading = car.heading;
    velocity.velocity = car.velocity;

    simulator.car_location_pub.publish(location);
    simulator.car_velocity_pub.publish(velocity);
    simulator.publish_car(simulator.n, location);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}