# Model documentation

# 1. Path planning algorithm 

The algorithm described here allows the control of an autonomous vehicle in a high-speed highway by enabling behavior selection and trajectory generation features. A class was created
containing the main properties (current lane and changing lane status) and methods required to reach the challenges described in the project rubric. For instance:

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
In the process the autonomous vehicle will speed up if the way ahead (same lane) is free of obstacles or will slow down in case a vehicle is too close (less than 30 Meters of distance) to the vehicle ahead, trying to match its speed. If the car is closer than 5 Meters in distance the autonomous vehicle will slow down with an acceleration of -8 m/s2.  

### 1.1.2 Transition to left/right lane 
The transition to the adjacent (left or right) lane will be enabled when (1) there is no vehicle in the adjacent lane behind it that will surpass its s value minus 35 Meters after 2 seconds **and** (2) there is no vehicle in the adjacent lane ahead that will be closer than 10 Meters after 2 seconds in a potential transition to the adjacent lane. When both conditions are met, the lane transition is possible and the `current_transition` flag is `false`, the lane value will be changed and `current_transition` flag will be set to true. Once the lane transition has been completed the `current_transition` is set to false again. 

The two main conditions used in this step are defined as follows:

#### (1) Free in lower left/right direction
It also tests whether the current car speed is enough to be ahead of the car by 35 Meters or more after 2 seconds.

``` Python
bool condition_prev_cars = ((car.s > check_car_s) && ((car.s+car.speed_ref*2)  < (check_car_s+check_speed_ms*2+35)));
```

#### (2) Free in upper left/right direction 
It also tests whether the current car speed is enough to be behind of the car by 10 Meters or more after 2 seconds.

```Python
bool condition_ahead_cars = ((car.s < check_car_s) && ((check_car_s-car.s) < 30) && (check_car_s+check_speed_ms*2 < car.s+car.speed_ref*2+10));
```

### 1.1.3 Fastest lane
The algorithm also detects the fastest lane by finding a lane that is empty ahead in a range of no more than 100 m or if, there is no empty lane, taking the lane where the closest vehicle ahead (no matter in which lane) to the autonomous vehicle has the biggest s value. The fastest lane identifier is used as an optimizer to define in which direction the vehicle should turn next in the case left or right lane transition are enabled. This feature comes handy when the vehicle ahead in the same lane is too close, there is a faster lane and the transition to that lane is possible given by the conditions in 1.1.2 or there is no vehicle too close in the current lane but there is a faster lane that is different to the current one.  

The output of this method will be to set the state of changing_lane according to the need of the autonomous vehicle on the track and the lane where the new trajectory should be projected to.

# 1.2 Trajectory generation

The method trajectory generation in the class ```PathPlanning``` calculates the best trajectory given the lane directive obtained from the behavior selection method. In the first step, we define a set of initial points for the new trajectory in such a way that we can guarantee a smooth transition. If the previous trajectory has too few points (less than 2), we will use the current x, y, and yaw parameters of the autonomous vehicle and calculate the previous point by subtracting the `cosine` and `sine` of the car yaw to `x` and `y` respectively. When the previous trajectory has more than 2 points we will use the last two points of it as the initial points of the new trajectory. 

Next, using **frenet** coordinates and the map waypoints we obtain the x and y coordinates of points 30, 60 and 90 Meters ahead in the `s` axis while following the map waypoints in the current lane. These five points are then transformed to the coordinate system of the car itself, with a translation given by the x,y position of the car and rotation using the car yaw. Once the transformation has been established, we use the spline function to interpolate all the points and guarantee transitions that are continuous, differentiable and smooth. 

Now, we create a new trajectory vector including all the points from the previous trajectory. Then we populate the new trajectory vector with values obtained from the spline until we reach a maximum of 50 points. The idea is to distribute these points in an x range from 0 to 30 Meters and y range from 0 to the value provided by `spline(30.0)` Meters. To reach the expected speed, we distribute these points according to the number `double N = target_distance / (0.02*car.speed_ref)` with the goal of evenly divide `target_distance` in N points. `target_distance` is the distance provided by the `x` range 30 Meters and `y` range `spline(30.0)`. Before adding each new value obtained from the spline to the new trajectory we need to transform them again to the global coordinate system. We do this by first rotating the points by minus car yaw (opposite angle), then we translate by adding the car `x` and `y` to each new spline point.  
