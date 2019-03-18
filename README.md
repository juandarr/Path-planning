# Model documentation

# 1. Path planning algorithm 

The algorithm described here allows the control of an autonomous vehicle in a high-speed highway by enabling behavior selection and trajectory generation features. A class was created
containing the main properties (current `lane`, `changing lane` status and `too_close`, for vehicles behind/ahead) and methods required to reach the challenges described in the project rubric. For instance:

- The car drives according to a speed limit of 50 miles per hour.
- Max acceleration and jerk don't exceed 10 m/s2 and 10 m/s3 respectively.
- The car doesn't have collisions.
- The car stays in its lane, except for the time when it is changing lanes.
- The car is able to change lanes.
- The car is able to drive at least 4.32 Miles without incident, meaning any of the previous conditions are satisfied. 

## 1.1 Behavior selection 

This method computes the best next action to be followed by the autonomous car given the car localization information and sensor fusion data from surrounding vehicles. The set of basic behaviors is as follows:
    - Keep the current lane
    - Transition to left lane
    - Transition to right lane

### 1.1.1 Keep the current lane 
The autonomous vehicle will speed up if the way ahead (same lane) is free of obstacles or will slow down in case a vehicle is too close (defined by the relative speed of vehicles plus 5 Meters of minimal distance) to the vehicle ahead, trying to match its speed. If the car is closer than 5 Meters in distance the autonomous vehicle will slow down with an acceleration of -9.5 m/s2, which is almost the minimum allowed of (-10m/s2). The lane will be kept when the fastest lane is the current lane or when a transition to adjacent lanes is blocked.  

### 1.1.2 Transition to left/right lane 
Transition to adjacent (left or right) lanes is allowed when (1) there is no vehicle in the adjacent lane lower direction that will surpass the minimum safe distance behind **and** (2) there is no vehicle in the adjacent lane upper direction that will be less than the minimum safe distance ahead. Relative speeds or vehicles are also considered in the calculation of the constraints. When both conditions are met for a particular lane and the `changing_lane` flag is `false`, the lane value will be changed accordingly and `changing_lane` is set to true. Once the lane transition has been completed the `current_transition` is set to false again. 

The conditions at the lower and upper lane directions are defined as follows:

#### (1) Blocked at lower left/right direction
It also tests whether the relative speed of vehicles is enough to maintain the minimum safe distance behind after 2 seconds.

```C++
// Blocked at lower direction
bool blocked_behind = !((car.s-closest_s_behind[temp_lane]+(car.speed_ref-closest_speed_behind[temp_lane]-9.5)*2.0) > min_safe_distance_behind);
```

#### (2) Blocked at upper left/right direction 
It also tests whether the relative speed of vehicles is enough to maintain the minimum safe distance ahead after 2 seconds.

```C++
// Blocked at upper direction
bool blocked_ahead = !((closest_s_ahead[temp_lane]-car.s+(closest_speed_ahead[temp_lane]+9.5-car.speed_ref)*2.0) > min_safe_distance_ahead);
```

### 1.1.3 Fastest lane
The algorithm also detects the fastest lane considering empty lanes ahead in a range of no more than 120 m or if, no empty lane is available, picking the lane where the closest vehicle ahead with largest s value is found (no matter which lane). The fastest lane value is used as an optimizer to define in which direction the vehicle should turn next in the case left or right lane transition are possible. This feature comes handy when the current lane is different from the fastest lane and (1) the transition to the fastest lane is possible given by the conditions in 1.1.2 or (2) the vehicle ahead in the same lane is too close and also the conditions in 1.1.2 are met. 

### 1.1.4 Speed transitions
The vehicle is expected to travel at the fastest speed when not being blocked by any adjacent vehicle. When a car is too close in the upper lane direction the vehicle will slow down with an acceleration enough to match the speed the car ahead and keep a minimum safe distance of 5 Meters. If the between vehicles is less than 5 Meters the vehicle will slow down with the maximum allowed acceleration of -9.5 m/s2. If a vehicle is too close to the car in the lower lane direction the vehicle will speed up trying to move farther from the car approaching. If this distance is less than the minimum safe distance behind the vehicle will speed up with a maximum acceleration of 9.5 m/s2. The default speed transition happens when there are no vehicles too close ahead or behind and the speed is less than the maximum allowed. In this final case, the vehicle will speed up with an acceleration of 5 m/s2.

### 1.1.5 Lane transition of adjacent cars
The algorithm includes logic to detect when cars ahead from adjacent lanes are moving towards the vehicle current lane. In this situation, the vehicle will keep a watch in the d value of this vehicle and when it is getting too close to the current lane will activate the `too_close` ahead (index 1) flag. This will result in the vehicle slowing down to keep the minimum safe distance ahead or changing the lane if conditions 1.1.2 are met. Here is the condition used to detect this state:
```C++
(closest_d > (4*lane-0.75) && closest_d<(4.75+4*lane))
```
`lane` is the current lane of the autonomous vehicle and `closest_d` the `d` value of the closes vehicle ahead. 

# 1.2 Trajectory generation

The method `trajectoryGeneration` in the class ```PathPlanning``` generates the best trajectory given the `lane` directive obtained from the behavior selection method. The logic to generate a new trajectory is as follows: 
1. We define a set of initial points for the new trajectory so we can guarantee a smooth transition. If the previous trajectory has too few points (less than 2), we will use the current x, y, and yaw parameters of the autonomous vehicle and calculate the previous point by subtracting the `cosine` and `sine` of the car yaw from `x` and `y` respectively. When the previous trajectory has more than 2 points we will use the last two points of it as the initial points of the new trajectory. 

2. Using **frenet** coordinates and the map waypoints we obtain the x and y coordinates of points 32, 64 and 96 Meters ahead in the `s` axis while following the map waypoints in the current lane. These five points are then transformed to the coordinate system of the car itself with a translation given by the x,y position of the car and rotation using the car yaw. Once the transformation has been established, we use the `Spline` function to interpolate all the points and guarantee transitions that are continuous, differentiable and smooth. 

3. We create a new trajectory vector including all the points from the previous trajectory. Then we populate the new trajectory vector with values obtained from the spline until we reach a maximum of 50 points. The idea is to distribute these points in an x range from 0 to 32 Meters and y range from 0 to the value provided by `spline(32.0)` Meters. To reach the expected speed, we distribute these points according to the number `double N = target_distance / (0.02*car.speed_ref)` with the goal of evenly divide `target_distance` in N points. `target_distance` is the distance provided by the `x` range 32 Meters and `y` range `spline(32.0)`. Before adding each new value obtained from the spline to the new trajectory we need to transform them again to the global coordinate system. We do this by first rotating the points by minus car yaw (opposite angle), then we translate by adding the car `x` and `y` to each new spline point. And that's it, an optimal trajectory is ready to command the vehicle motion!  

# Instructions

This project involves the Udacity Self Driving Car  Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). The simulator serves as a visual output of the logic performed in the repository program, which creates a server that uses the simulation as a front end. 

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems (`install_ubuntu.sh` and `install_mac.sh`). For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

- `./build.sh`
- `./run.sh`
 
 Any executables can be removed with the following script:

 - `./clean.sh`

After running the program, open the siumulator. You should get a `connected` message in the terminal output, meaning the simulator is connected to the server program and is ready to run. 

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

# Project definition

The goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 

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

---

# License

Copyright (c) Udacity and Juan David Rios. All rights reserved.

Licensed under the [MIT](https://github.com/juandarr/Path-Planning/blob/master/LICENSE) License.