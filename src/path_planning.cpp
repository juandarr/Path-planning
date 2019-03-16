/**
 * Path Planning class code
 * Implementation of behavior planning and trajectory generation
 * for the tour of an autonomous vechile in a high speed highway
 * 
 * Created on: March 15, 2019
 * Author: Juan Rios
 */

#include "path_planning.h"

#include <vector>
#include <iostream>
#include "json.hpp"
#include "helpers.h"
#include "spline.h"
#include <limits>

using std::cout;
using std::endl;
using std::vector;
using nlohmann::json;

void PathPlanning::behaviorSelection(Car &car, json &sensor_fusion, int prev_size){

    // Check if the car is in lane transition state and set the state false if the car is inside the expected new lane
    if ((car.d < (2+4*lane+1)) && (car.d > (2+4*lane-1)) && changing_lane) {
        changing_lane = false;
        cout << "Lane transition completed!" << endl;
        if (collision_direction != 0) {
            collision_direction = 0;
            cout << "Avoid collision disabled!" << endl;
        }
        
    }

    /**
     * These variables are used to store the s value of closest cars in each direction. 
     * Using these values we can identify the lane availability for different transitions
     */
    // Closest vehicle in the upper direction
    double closest_s_up = std::numeric_limits<double>::max();
    // Closest vehicle in the upper left direction
    double closest_s_left_up = std::numeric_limits<double>::max();
    // Closest vehicle in the lower left direction
    double closest_s_left_down = std::numeric_limits<double>::min();
    // Closest vehicle in the upper right direction
    double closest_s_right_up = std::numeric_limits<double>::max();
    // Closest vehicle in the lower right direction
    double closest_s_right_down = std::numeric_limits<double>::min();

    /**
     *  Variables used to flag the availability of car transition towards upwards,
     *  left or right lane state
     */ 
    // The vehicle ahead is too close
    bool too_close = false;
    // Car doesn't have or will have dangerous obstacles in the upper left direction
    bool left_clear_up = true;
    // Car doesn't have or will have dangerous obstacles in the lower left direction
    bool left_clear_down = true;
    // Car doesn't have or will have dangerous obstacles in the upper right direction
    bool right_clear_up = true;
    // Car doesn't have or will have dangerous obstacles in the lower right direction
    bool right_clear_down = true;

    // Transition speed when the car ahead is slow and we need to gradually reduce speed
    double transition_vel = car.speed_ref;
    
    // Vector too store the closest cars from the s value of the car ahead, in each of the index lanes 0:left, 1:center and 2:right
    vector<double> closest_s_lanes = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Set current lane as fastest lane
    int fastest_lane = lane;
    

    /**
     * This loop is used to detect whether (1) there is a car too close ahead, (2) the left/right lanes
     * are free of obstacles and (3) store the closest cars s values in each lane. (3) is used to later set
     * the fastest_lane value and define that lane as the preferred direction during behavior selection
     */ 
    for (unsigned int i = 0; i < sensor_fusion.size(); ++i) {
        
        // d value of vehicle with index i
        float d = sensor_fusion[i][6];

        // X component of speed for vehicle with index i
        double vx = sensor_fusion[i][3];
        // Y component of speed for vehicle with index i
        double vy = sensor_fusion[i][4];
        // Speed of vehicle with index i
        double check_speed = sqrt(vx*vx + vy*vy);
        // S position of vehicle with index i
        double check_car_s = sensor_fusion[i][5];
        
        // Updated s position of car considerin the time step and size of previous path vector 
        check_car_s += check_speed*0.02*prev_size;

        // Convert vehicle speed from Miles per hour to Meters per Second
        double check_speed_ms = check_speed*1609.0/3600.0;

        /**
         * Detects closest s car position to the vehicle in each lane. We are interested in those closest cars where
         * their s position is bigger than the s position of the autonomous vehicle.
         */ 
        if (car.s < check_car_s && (check_car_s-car.s)<100 && d > 0 && d < 12) {
            int lane_temp = floor(d/4);
            if (check_car_s < closest_s_lanes[lane_temp]) {
                closest_s_lanes[lane_temp] = check_car_s;
            } 
        }  

        // Identify whether cars ahead in the current lane are too close
        if (d > (2+4*lane-2) && d < (2+4*lane+2)) {
            // Car is ahead the autonomous vehicles is the distances is less than 30 Meters
            if ((car.s < check_car_s) && (check_car_s-car.s)< 30) {
                // Car is closer to the autonomous vehicle than the previous closest s value
                if (closest_s_up > check_car_s) {
                    closest_s_up = check_car_s;
                    // If car is too close do some action      
                    too_close = true;
                    transition_vel = check_speed_ms;
                    
                }
            }
        // Identify whether right lane to the current one is free of obstacles in the upper and lower right directions
        } else if  (d > (2+4*lane+2) && d < (2+4*lane+6)) {
            if (lane<2) {
                if (abs(d-car.d) < 2.5 && (abs(car.s-check_car_s)<20) && collision_direction==0) {
                    collision_direction = 1;
                    cout << "Activating action to avoid collision with vehicle coming from right lane!" << endl;
                    transition_vel = car.speed_ref / 2.0;
                } else if (collision_direction!=0) {
                    collision_timer += 1;
                    if (collision_timer > 60) {
                        collision_direction = 0;
                        cout << "Avoid collision disabled!" << endl;
                        collision_timer = 0;
                    }
                } 

                // Free in lower right direction. It also tests whether the current car speed is enough to be ahead of the car by 35 Meters or more after 2 seconds.
                bool condition_prev_cars = ((car.s > check_car_s) && ((car.s-check_car_s) < 10) && ((car.s+car.speed_ref*2.0)  < (check_car_s+check_speed_ms*2.0+35)));
                // Free in upper right direction. It also tests whether the current car speed is enough to be behind of the car by 10 Meters or more after 2 seconds.
                bool condition_ahead_cars = ((car.s < check_car_s) && ((check_car_s-car.s) < 20) && (check_car_s+check_speed_ms*2.0 < car.s+car.speed_ref*2.0+30));  
                
                if (condition_prev_cars) {
                    if (closest_s_right_down < check_car_s) {
                        // Set availability as false in the lower right direction for right lane transition
                        right_clear_down = false;
                        closest_s_right_down = check_car_s;
                    }
                }
                if (condition_ahead_cars) {
                    if (closest_s_right_up > check_car_s) {
                        // Set availability as false in the upper right direction for right lane transition
                        right_clear_up = false;
                        closest_s_right_up = check_car_s;
                    }
                }
            }
        // Identify whether left lane to the current one is free of obstacles in the upper and lower left directions
        } else if  (d < (2+4*lane-2) && d > (2+4*lane-6)) {
            if (lane>0) {
                if (abs(d-car.d) < 2.5 && (abs(car.s-check_car_s)<20) && collision_direction==0) {
                    collision_direction = -1;
                    cout << "Activating action to avoid collision with vehicle coming from left lane!" << endl;
                    transition_vel = car.speed_ref / 2.0;
                } else if (collision_direction!=0) {
                    collision_timer += 1;
                    if (collision_timer > 60) {
                        collision_direction = 0;
                        cout << "Avoid collision disabled!" << endl;
                        collision_timer = 0;
                    }
                } 
                
                // Free in lower left direction. It also tests whether the current car speed is enough to be ahead of the car by 35 Meters or more after 2 seconds.
                bool condition_prev_cars = ((car.s > check_car_s) && ((car.s-check_car_s) < 10) &&  ((car.s+car.speed_ref*2.0)  < (check_car_s+check_speed_ms*2.0+35)));
                // Free in upper left direction. It also tests whether the current car speed is enough to be behind of the car by 10 Meters or more after 2 seconds.
                bool condition_ahead_cars = ((car.s < check_car_s) && ((check_car_s-car.s) < 30) && (check_car_s+check_speed_ms*2.0 < car.s+car.speed_ref*2.0+30));
                
                if (condition_prev_cars) {
                    if (closest_s_left_down < check_car_s) {
                        // Set availability as false in the lower left direction for left lane transition
                        left_clear_down = false;
                        closest_s_left_down = check_car_s;
                    }
                }
                if (condition_ahead_cars) {
                    if (closest_s_left_up > check_car_s) {
                        // Set availability as false in the upper left direction for left lane transition
                        left_clear_up = false;
                        closest_s_left_up = check_car_s;
                    }
                }
            }
        }
    }

    /**
     * Here we define the fastest lane. The fastest lane is the one that is free of cars ahead
     * or the maximum s distance ahead to the autonomous car
     */ 
    // If current lane is free of cars ahead, set that lane as the fastest one
    if (closest_s_lanes[lane]==std::numeric_limits<double>::max()) {
        fastest_lane = lane;
    // Else, look for the maximum s distance. 
    } else {
        double s_temp = std::numeric_limits<double>::min();
        for (unsigned int i = 0; i < closest_s_lanes.size(); ++i) {
            if (s_temp < closest_s_lanes[i]) {
                fastest_lane = i;
                s_temp = closest_s_lanes[i];
                // If a lane next to the current lane is free, define it as the fastest one and break the loop
                if (((i == (lane-1)) || (i == (lane+1))) && (closest_s_lanes[i]==std::numeric_limits<double>::max())) break;
            }
        }
    } 

    /**
     * Behavior selection: Change lane or stay in lane
     */ 
    // If car ahead is too close consider changing lane.
    // Use fastest lane direction as the preferred lane transition direction
    if ((too_close  && car.speed > 20.0) || (collision_direction!=0 && lane==1)) {
        if (fastest_lane < lane) {
            // If current lane is center or right, left lane is free for lane change and changing_lane flag is false       
            if (lane>0 && left_clear_up && left_clear_down && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Left." << std::endl;
                lane -= 1;
                changing_lane = true;
            // Else, consider the right lane contraints
            } else if (lane<2 && right_clear_up && right_clear_down && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Right" << std::endl;
                lane += 1;
                changing_lane = true;
            }
        } else {
            // If current lane is center or left, right lane is free for lane change and changing_lane flag is false   
            if (lane<2 && right_clear_up && right_clear_down && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Right." << std::endl;
                lane += 1;
                changing_lane = true;
            // Else, consider the left lane contraints
            } else if (lane>0 && left_clear_up && left_clear_down && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Left." << std::endl;
                lane -= 1;
                changing_lane = true;
            } 
        }
    // Else, there are not obstacles ahead but the fastest lane is not current lane
    } else if ((lane != fastest_lane)) {
        // Consider moving in the direction of the fast lane 
        if (fastest_lane < lane) {       
            if (lane>0 && left_clear_up && left_clear_down && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Left." << std::endl;
                lane -= 1;
                changing_lane = true;
            } 
        } else {
            if (lane<2 && right_clear_up && right_clear_down && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Right." << std::endl;
                lane += 1;
                changing_lane = true;
            } 
        }
    }

    /**
     * Behavior selection: Change speed.  If car ahead is too close reduce the speed, else increase speed
     */ 
    if (too_close)
    {
        // If car is 5 meters or less ahead reduce speed with an acceleration of 8m/s2 every 0.02 s
        if (abs(closest_s_up - car.s) < 15) {
            if (car.speed_ref > 0.0) {
                car.speed_ref -= 0.16;
            }
        // Else reduce speed with an acceleration to reach the speed of car ahead after 4 seconds
        } else {
            car.speed_ref -= ((car.speed_ref - transition_vel)*0.02/2.0);
        }
    // If autonomous vehicle is not too close to cars ahead, speed up
    } else if ((collision_direction!=0) && !changing_lane) {
        if (car.speed_ref > 0.0 && (abs(closest_s_lanes[lane]-car.s) < 15 || abs(closest_s_lanes[lane+collision_direction]-car.s) < 15)) {
            cout << "Slowing down instead of lane transition to avoid collision!" << endl;
            car.speed_ref -= 0.16;
        }
    } else if (car.speed_ref < 22.12375) {
        car.speed_ref += 0.10;
    }
};

vector<vector<double>> PathPlanning::trajectoryGeneration(Car &car, vector<double> previous_path_x, vector<double> previous_path_y, \
                        vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y) {
    
    vector<vector<double>> path_vals;

    // Create a list of widely spaced (x,y) waypoints, evenly spaces at 30 m
    // We will interpolate this points using the spline method
    vector<double> ptsx;
    vector<double> ptsy;

    // Reference x, y and yaw states
    // We will reference the current car position or we will access the last path end point
    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);

    // If the previous path is almost empty, use the car as starting reference
    int prev_size = previous_path_x.size();
    if ( prev_size < 2) {
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);  

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car.y);
    } 
    // Use previous path points as starting reference
    else {
        
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        
        // Define current vehicle orientation from its two previous point locations
        ref_yaw = atan2((ref_y-ref_y_prev),(ref_x-ref_x_prev));

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Add evenly spaces points 30 m ahead of the starting point in frenet coordinates
    vector<double> next_wp0 = getXY(car.s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car.s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car.s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    /**
     * Apply translation and rotation transformation to move 
     * points from the global coordinate system to the local car coordinate
     * system
     */ 
    for (unsigned int i = 0; i < ptsx.size(); ++i) {
        // Shift car reference to origin and angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }

    // Create a spline  
    tk::spline s;

    // Set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Start with all of the previous path points of last time
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (unsigned int i =0; i < previous_path_x.size(); ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate how to break up spline points so that we travel at our desired reference speed
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_distance = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0.0;    

    // Define N, the number of evenly transitions expected to guarantee an average speed ot ref_vel in the next
    // 30 Meters
    double N = target_distance / (0.02*car.speed_ref);
    double x_point;
    double y_point;

    for (unsigned int i = 1; i < 50-previous_path_x.size(); ++i) {

        x_point = x_add_on + target_x/N;
        y_point = s(x_point);  

        x_add_on = x_point;

        double x_temp = x_point;
        double y_temp = y_point;

        x_point = (x_temp*cos(ref_yaw) - y_temp*sin(ref_yaw));
        y_point =  (x_temp*sin(ref_yaw) + y_temp*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    path_vals.push_back(next_x_vals);
    path_vals.push_back(next_y_vals);

    return path_vals;
};
