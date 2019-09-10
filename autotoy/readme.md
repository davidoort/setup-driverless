#Autotoy
In this project a full driverless-software pipeline is simulated. 
The goal of this project is to learn ROS.

## High level project flow.
First a track is generated, next the cones along the track are generated. 
This track is the input for the simulation.
The initial car location and rotation together with all cone locations is passed to a camera simulator.
This simulator chooses a subset of cones that the car sees.
These cone locations are passed to the track finder which tries to find the centre-line between the cones.
Next, the car-controller will take the current position and rotation of the car together with the centre line and come up with the rotation of the wheel.
This is passed back to the simulator which will move the virtual car accordingly.
The new location of the car is passed to the camera simulation and the whole thing will start again.
Graphically, the above flow looks like this: 

![](img/autotoy-sequence.png)
```https://sequencediagram.org
title Autotoy project

Simulator->Track generator:track/Generate
Simulator<--Track generator:track/Track
Track generator->Cone placer:track/GenerateCones
Track generator<--Cone placer:track/Cones

loop simulation
  Simulator->Camera simulation: car/Location
  Camera simulation->Track Finder: car/VisableCones
  Track Finder->Car Control:car/TrackCentreLine
  Car Control->Simulator: car/Steering
end
```

## Components specification
Here every component's interface is further specified.
This information should be enough to implement each node independently.

### Message types
*track/Point.msg*
```
int8 x
int8 y
```

*track/Cones.msg*
```
track/Point[] cones
```

*track/Track.msg*
```
track/Point[] centreline
track/Point[] leftcones
track/Point[] rightcones
```

*track/Line.msg*
```
track/Point[] points
```

*car/Location.msg*
```
track/Point location
int8 rotation
```

### Topic definitions
| Topic name | Description | Message type | 
|---|---|---|
| `/track` | The full track will be published here at the start of the simulation. | `track/Track.msg` |
| `/car/camera` | The cones visible of the cars will be published here. | `track/Cones.msg` |
| `/car/location` | The location of the car will be published here. | `car/Location.msg` |
| `/car/targetline` | The line that the car should be following. | `track/Line.msg` |

### Nodes

### Track generator
* package: track
* node name: generator
* node type: [service](http://wiki.ros.org/srv)
* service name: `/track/generate`
definition of `Generator.srv`:
```
int8 tracklength
Point start
---
track/Point[] centreline
track/Point[] leftcones
track/Point[] rightcones
```

#### Track cone placer
* package: track
* node name: coneplacer
* node type: [service](http://wiki.ros.org/srv)
* service name: `track/conesplacer`
* service definition `ConePlacer.srv`
```
track/Point[] route
---
track/Point[] leftcones
track/Point[] rightcones
```

#### Car camera simulation
* package: car
* node name: camerasimulator
* node type: [topic](http://wiki.ros.org/Topics)
* Listens on topic `/track` to receive a new track. It will store this track in memory for future usage.
* Listens on topic `/car/location` to receive the location of the car. 
  When received, it will find the cones that are currently visible and send the result to the `/car/camera` topic. 

#### Car track finder
* package: car
* node name: trackfinder
* node type: [topic](http://wiki.ros.org/Topics)
* Listens on topic `/car/camera` and will try to find the centre line of the track. The result will be sent to `/car/targetline` topic.

#### Car controller
* package: car
* node name: controller
* node type: [topic](http://wiki.ros.org/Topics)
* Listens on topic `/car/targetline` for updates on the projected line and listens on the `/car/location` for the location and rotation of the car.
  This node calculates the new rotation of the car every time a message is received on one of the previously mentioned topics.
  The result is published on `/car/steering`.

#### Simulator
* package: simulator
* node name: main
* node type: [topic](http://wiki.ros.org/Topics)
When the node starts it requests a new track from the `/track/generate` service.
The new track is published on `/track`.
Next, the start-position of the car is published on `/car/location`.
Now it waits for messages on the `/car/steering` topic.
When a steering-event is received the value is applied to the rotation of the car and the car is moved 5 pixels in the direction the of the car. 

## Development
To build the project run `catkin_make`.

## Running
To run the project, start ros and run every node. Start the simulator last. Each should be started from a separate console:
```
roscore

rosrun track generator
rosrun track coneplacer
rosrun car camerasimulator
rosrun car trackfinder
rosrun car controller

rosrun simulator main
```
