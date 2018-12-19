# CarND-Path-Planning-Project
Path Planner for Autonomous Vehicle Driving around a Virtual Highway.  
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every 0.02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a 0.02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Model Documentation

The path planner model in this project is a discrete system that runs two main steps sequentially and periodically. The first step is behavior planning that takes as an input the state (location and speed) of the ego vehicle and the other vehicles on the road and outputs the next state of the car. The second step implements the trajectory generation unit whose inputs are the previous location of the car as well as the desired speed and lane from the behavior planner and its outputs are a set of consecutive <X,Y> points in the world-coordinate system. 

1. The behavior planner in this project is effectively a simple finite-state machine (FSM) with two states. The first state is basically a **"keep-lane" state** where the car will stay in its lane as long as: (1) it is traveling at maximum target speed (~50 MPH) with no traffic ahead of it, or (2) it cannot safely change lanes when it runs into slower traffic (< 50 MPH) in its lane. When slow traffic is encountered in this state, the car's speed will be set down to the speed of the car ahead of it. The rate of velocity change (i.e, deceleration or braking) is not allowed to go below -10 m/s^2. The ego vehicle can gradually speed up if the vehicle in front of it increased its speed or changed lanes. The new speed will be of the vehicle ahead (if any) or the maximum target speed, whichever is lower. Again, the rate of velocity increase should not exceed 10 m/s^2. The functions **brake** and **accelerate** implement the logic of speed change given the current speed and maximum deceleration and acceleration rates, respectively.  

If the ego vehicle can safely change lanes when obstructed by slow traffic, it enters into the **"lane-change" state**. The vehicle prioritizes switching to the left lane over the right lane as it is the customary/legal behavior in driving on public roads. Before switching lanes, the vehicle first checks if it is not in the left-most (or right-most) lane before turning left (or right). If it is not, it then checks if there will be no slow traffic in the new lane. Slow traffic detection in the proposed lane is similar to slow traffic detection in the current host lane. The only difference here is that the ego vehicle also checks if there is a vehicle cruising by its side that it may collide with if a lane change is made. The function **detect_car_on_shoulder** uses the ego vehicle's and other vehicles' Frenet "s" coordinates as well as their velocities to detect if there will be any close contact with a vehicle on the side upon a lane change. A decision of lane change is made if both of these checks reveal that there are no slow traffic in the new lane and no vehicles driving approximately parallel to the ego vehicle in the proposed lane. Executing speed and/or lane change is facilitated by the subsequent trajectory generation step as described below.        
2. The second step of the path planner involves translating the proposed speed and lane into a sequence of carefully-spaced waypoints in the world-coordinate system. In this project, we are provided with sparse waypoints of the center of the road irrespective of the intended speed and lane. Since the car needs to drive along the proposed lane with some lateral shift from the center of the road and moves between any two subsequent waypoints on that lane in 0.02 seconds, a new set of waypoints have to be generated and spaced with respect to each other such that the car drives on the intended lane and as close to the proposed speed as possible without exceeding the maximum acceleration and jerk limits. Meeting the maximum acceleration limit constraint is already handled when computing the new speed from the current speed in the **accelerate** and **brake** functions. Creating smooth trajectories using the spline function was really helpful in making the car follow the sparse waypoints smoothly without exceeding the maximum allowed jerk. The spline is created from a few anchor <X,Y> points along the planned path as follows:
    1. Two starting points are the last two points from the previous path points to ensure smooth transition between path-point-generation iterations.
    2. I also create additional anchor points with varying distances (30, 60, 90, etc., meters) away from the vehicle's location on the s-coordinate along the current or proposed lane. These are basically the middle and end points of the spline I am fitting. Well-spaced anchor points ensure minimal jerking in case of lane change and/or curved road.
    3. The anchor points are then converted to the vehicle coordinate system and a spline curve is fitted using a self-contained C++ Spline interpolation library http://kluge.in-chemnitz.de/opensource/spline/.  
    4. The resulting spline's equation is then used to compute the lateral position y for every longitudinal position x along the path. As stated above, these points are spaced apart such that the resulting vehicle speed when transitioning between the points is approximately the speed proposed by the behavioral planner. To achieve this spacing, the curve was linearized and then broken into segments where the length of each segment is calculated based on the proposed speed and the time the car takes to finish each segment (i.e., the time to move from one point to the next which is set to be 0.02 seconds in this project). 
    5. The longitudinal and lateral points are then converted back from the vehicle to the world coordinate system and sent to the simulator to use for moving the car. 

These two steps of behavior planning and trajectory generation is repeated over and over until the simulator is closed.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```



