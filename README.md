[state_machine]: ./pictures/state_machine.png "State Machine"
[vehicle_detected_left]: ./pictures/vehicle_detected_left.png "Vehicle Detected Left"
[change_lane_right]: ./pictures/change_lane_right.png "Lane Change Left"
[change_lane_left]: ./pictures/change_lane_left.png "Lane Change Right"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided along with a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

---

# Implementation Details

The following sections describe the implementation details of the project.

## Lane Change Vehicle Detection

Our car that travels along the highway will now be named Ego to avoid confusion when referring to other cars on the highway.

When Ego is travelling along the highway, it will continuously monitor if there is another car that is slower in the same lane.

If there is another car in the same lane and Ego is getting too close, Ego will see if there is a lane available for a lane change. The lanes next to Ego will be checked to see if there is a vehicle that's too close ahead of behind of Ego. The check is to make sure that it is meaningful for Ego to change to that lane. If there's a clear lane with vehicles at an acceptable distance ahead and behind Ego's current position, Ego will change to that lane. All of this logic will be captured by a finite state machine.

An example of this can be seen in the image below where Ego is approaching the vehicle ahead on the lane. Ego is deciding to change lanes. However, Ego detects a car on the left hand side and no cars on the right lane. Ego will then lane change to the right lane.

![alt text][vehicle_detected_left]

## Finite State Machine
A finite state machine was implemented to dictate vehicle behavior on the highway. A diagram of the of the finite state machine can be found below.

There are a total of 5 states that will be discussed in the following sections.

### 1. Keep lane

The keep lane state will keep Ego in its current lane. It will monitor for the car speed and keep the car under the speed limit. If there is a car ahead, this state will strive to maintain a safe speed and distance away from the car. The finite state machine will then decide whether it is safe and feasible to pass the car ahead. The possible successive states are prepare for lane change left or prepare for lane change right.

### 2. Prepare for lane change left

When Ego needs to pass a vehicle on the left, it will transition to the prepare for lane change left state. This state will determine if it is safe to pass on the left lane. If there are cars ahead or behind on left lane, the finite state machine will check if it is at an appropriate distance away from Ego.  If it is no longer safe to pass on the right lane, the state will transition back to the keep lane state.

### 3. Prepare for lane change right

When Ego needs to pass a vehicle on the right, it will transition to the prepare for lane change right state. Similar to the prepare for lane change left state, this state will determine if it is safe to pass on the right lane. If there are cars ahead or behind on left lane, the finite state machine will check if it is at an appropriate distance away from Ego. If it is no longer safe to pass on the right lane, the state will transition back to the keep lane state.

### 4. Lane change left

This state will execute when it is determined that it is safe for Ego to pass on the left lane. Ego remains in this state until it has successfully lane changed. The lane change status is checked by monitoring Ego's frenet coordinate d until it reaches the goal lane's d boundaries. Once Ego has successfuly lane changed, Ego's state will return to the keep lane state.

An image of Ego changing lanes to the left can be seen below:

![alt text][change_lane_left]

### 5. Lane change right

Same as lane change left.

An image of Ego changing lanes to the right can be seen below:
![alt text][change_lane_right]

## Path Planning

The path planner determines how to execute the actions determined by the finite state machine. The planner creates the waypoints for Ego to follow the highway lane and to change lanes.

### Lane Calculation
Telemetry data is read in at 0.02 second intervals. The data contains the Frenet coordinate data of each car including Ego. Frenet coordinate data contains the s value which is the distance alone the lane of the highway and and the d value which is the the distance measured from the from the center line of the highway.

Each highway lane is has a width of 4 meters. By knowing the desired car lane from the finite state machine, the required d value can be calculated.

### Trajectory Calculation

The trajectory planning is used to determine the next waypoints the car needs to follow.

The next waypoints for Ego to follow can be determined by using the previous waypoints, Ego's current coordinates and the next three waypoints that are 30, 60 and 90 meters ahead. The waypoints are then converted into XY coordinates in order to be interpreted by the simulator.

The waypoint coordinates are then connected by creating a spline. An external spline library is used to this.

---

## Future Features

There are some additional features that should be implemented in the future to better Ego's performance. The additional features should be added are listed below:

  1. When in the constant speed state, set Ego's speed to the car ahead's speed to maintain constant speed
  2. Let Ego search for gaps between cars in other lanes to select better lane changing routes
  3. Implement a cost function for Ego to decide which lane to take

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
