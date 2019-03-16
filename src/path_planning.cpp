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

    }

    /**
     *  Variables used to flag the availability of vehicle transition to 
     *  left or right lane state
     */ 
     // Vehicle is free of obstacles in the lower (index 0) and upper (index 1) left direction
    vector<bool> left_clear = {true , true};
    // Vehicle is free of obstacles in the lower (index 0) and upper (index 1) right direction
    vector<bool> right_clear = {true , true};
    

    /**
     *  Variables used to flag potential collision alerts
     */ 
    // The vehicle ahead is too close
    bool too_close_up = false;
     // The vehicle behind is too close
    bool too_close_down = false;
     // The vehicle to the left is too close
    bool too_close_left = false;
     // The vehicle to the right is too close
    bool too_close_right = false;


    // Transition speed when the car ahead is slow and we need to gradually reduce speed
    double transition_vel = car.speed_ref;
    
    // Store the s value of closest cars to vehicle that are ahead. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_s_ahead = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Store the s value of closest cars to vehicle form behind in each lane. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_s_behind = {std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
    
    // Store the s value of closest cars to vehicle that are ahead. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_speed_ahead = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Store the s value of closest cars to vehicle form behind in each lane. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_speed_behind = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    
    // Store the s value of closest cars to vehicle that are ahead. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_d_ahead = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Store the s value of closest cars to vehicle form behind in each lane. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_d_behind = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

    // Set current lane as fastest lane
    int fastest_lane = lane;
    

    /**
     * This loop is used to detect whether (1) there is a car too close ahead, (2) the left/right lanes
     * are free of obstacles and (3) store the closest cars s values in each lane. (3) is used to later set
     * the fastest_lane value and define that lane as the preferred direction during behavior selection
     */ 
    for (unsigned int i = 0; i < sensor_fusion.size(); ++i) {

        // Identifier
        int id = sensor_fusion[i][0];

        // X component of speed for vehicle with index i
        double vx = sensor_fusion[i][3];
        // Y component of speed for vehicle with index i
        double vy = sensor_fusion[i][4];
        // Speed of vehicle with index i
        double speed = sqrt(vx*vx + vy*vy);
        // S position of vehicle with index i
        double s = sensor_fusion[i][5];
        // d value of vehicle with index i
        float d = sensor_fusion[i][6];
        
        // Updated s position of car considering the time step and size of previous path vector 
        s += speed*0.02*prev_size;

        /**
         * Detects closest cars to the autonomous vehicle and their d, speed values in each lane. We are interested in the closest cars 
         * ahead and behind the autonomous vehicle
         */ 
        if (abs(s-car.s)<100 && d > 0 && d < 12) {
            int lane_temp = floor(d/4);
            if (car.s < s) {
                if (s < closest_s_ahead[lane_temp]) {
                    closest_s_ahead[lane_temp] = s;
                    closest_d_ahead[lane_temp] = d;
                    // Convert vehicle speed from Miles per hour to Meters per Second
                    closest_speed_ahead[lane_temp] = speed*1609.0/3600.0;
                } 
            } else {
                if (s > closest_s_behind[lane_temp]) {
                    closest_s_behind[lane_temp] = s;
                    closest_d_behind[lane_temp] = d;
                    // Convert vehicle speed from Miles per hour to Meters per Second
                    closest_speed_behind[lane_temp] = speed*1609.0/3600.0;
                } 
            }
        }  

    }
    
    // Car is ahead the autonomous vehicles is the distances is less than 20 Meters
    if ((closest_s_ahead[lane]-car.s)< 20) {
        // If car is too close do some action      
        too_close_up = true;
        transition_vel = closest_speed_ahead[lane];
    } 

    if ((car.s-closest_s_behind[lane])< 20) {
        too_close_down = true;
    }
    
    vector<int> lane_transition = {-1 , 1};

    // Identify whether right or left lanes are free of obstacles in the upper and lower right/left directions
    for (unsigned int i = 0; i < lane_transition.size(); ++i) {
        
        int new_lane = lane + lane_transition[i];
        
        if (new_lane >= 0 && new_lane <=2) {
            
            // Free in lower right direction. It also tests whether the current car speed is enough to be ahead of the car by 35 Meters or more after 2 seconds.
            bool condition_behind_cars = (((car.s-closest_s_behind[new_lane]) < 27.5) && \
                                        ((car.s+car.speed_ref*2.0)  < (closest_s_behind[new_lane]+closest_speed_behind[new_lane]*2.0+35.0)));
            
            // Free in upper right direction. It also tests whether the current car speed is enough to be behind of the car by 10 Meters or more after 2 seconds.
            bool condition_ahead_cars = (((closest_s_ahead[new_lane]-car.s) < 25.0) && \
                                        (closest_s_ahead[new_lane]+closest_speed_ahead[new_lane]*2.0 < car.s+car.speed_ref*2.0+30.0));  
           

            for (unsigned j = 0; j < right_clear.size(); ++j) {
                if (condition_behind_cars) {
                    if (lane_transition[i] == -1) {
                        // Set availability as false for left lane transition in lower left direction
                        left_clear[0] = false;
                    } else {
                        // Set availability as false for right lane transition in lower right direction
                        right_clear[0] = false;
                    }
                }
                if (condition_ahead_cars) {
                    if (lane_transition[i] == -1) {
                        // Set availability as false for left lane transition in upper left direction
                        left_clear[1] = false;
                    } else {
                        // Set availability as false for right lane transition in upper right direction
                        right_clear[1] = false;
                    }   
                }    
            }
        }
    }
    
    /*
    if (abs(d-car.d) < 3.10 && (check_car_s > car.s) && ((check_car_s-car.s)<20) && (collision_direction==0)) {
            id_car_collision = id;
            d_car_collision = d;
            collision_direction = 1;
            cout << "Activating action to avoid collision with vehicle coming from right lane!" << endl;
            transition_vel = car.speed_ref / 2.0;
        } else if (collision_direction!=0) {
            collision_timer += 1;
            if (collision_timer > 180) {
                collision_direction = 0;
                cout << "Avoid collision disabled!" << endl;
                collision_timer = 0;
            }
        } 
    */

    /**
     * Here we define the fastest lane. The fastest lane is the one that is free of cars ahead
     * or the maximum s distance ahead to the autonomous car
     */ 
    // If current lane is free of cars ahead, set that lane as the fastest one
    if (closest_s_ahead[lane]==std::numeric_limits<double>::max()) {
        fastest_lane = lane;
    // Else, look for the maximum s distance. 
    } else {
        double s_temp = std::numeric_limits<double>::min();
        for (unsigned int i = 0; i < closest_s_ahead.size(); ++i) {
            if (s_temp < closest_s_ahead[i]) {
                fastest_lane = i;
                s_temp = closest_s_ahead[i];
                // If a lane next to the current lane is free, define it as the fastest one and break the loop
                if (((i == (lane-1)) || (i == (lane+1))) && (closest_s_ahead[i]==std::numeric_limits<double>::max())) break;
            }
        }
    } 

    /**
     * Behavior selection: Change lane or stay in lane
     */ 
    // If car ahead is too close consider changing lane.
    // Use fastest lane direction as the preferred lane transition direction
    if ((too_close_up  && car.speed > 20.0)) {
        if (fastest_lane < lane) {
            // If current lane is center or right, left lane is free for lane change and changing_lane flag is false       
            if (lane>0 && left_clear[1] && left_clear[0] && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Left." << std::endl;
                lane -= 1;
                changing_lane = true;
            // Else, consider the right lane contraints
            } else if (lane<2 && right_clear[1] && right_clear[0] && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Right" << std::endl;
                lane += 1;
                changing_lane = true;
            }
        } else {
            // If current lane is center or left, right lane is free for lane change and changing_lane flag is false   
            if (lane<2 && right_clear[1] && right_clear[0] && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Right." << std::endl;
                lane += 1;
                changing_lane = true;
            // Else, consider the left lane contraints
            } else if (lane>0 && left_clear[1] && left_clear[0] && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Left." << std::endl;
                lane -= 1;
                changing_lane = true;
            } 
        }
    // Else, there are not obstacles ahead but the fastest lane is not current lane
    } else if ((lane != fastest_lane)) {
        // Consider moving in the direction of the fast lane 
        if (fastest_lane < lane) {       
            if (lane>0 && left_clear[1] && left_clear[0] && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Left." << std::endl;
                lane -= 1;
                changing_lane = true;
            } 
        } else {
            if (lane<2 && right_clear[1] && right_clear[0] && !changing_lane) {
                std::cout << "Fastest lane: " << fastest_lane << ", going Right." << std::endl;
                lane += 1;
                changing_lane = true;
            } 
        }
    }

    /**
     * Behavior selection: Change speed.  If car ahead is too close reduce the speed, else increase speed
     */ 
    if (too_close_up)
    {
        // If car is 5 meters or less ahead reduce speed with an acceleration of 8m/s2 every 0.02 s
        if (abs(closest_s_ahead[lane] - car.s) < 10) {
            if (car.speed_ref > 0.0) {
                car.speed_ref -= 0.18;
            }
        // Else reduce speed with an acceleration to reach the speed of car ahead after 4 seconds
        } else {
            car.speed_ref -= ((car.speed_ref - transition_vel)*0.02/4.0);
        }
    // If autonomous vehicle is not too close to cars ahead, speed up
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
