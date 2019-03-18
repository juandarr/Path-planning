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

void PathPlanning::behaviorSelection(Car &car, json &sensor_fusion){

    // Check if the car is in lane transition state and set the state false if the car is inside the expected new lane
    if ((car.d > (2+4*lane-1)) && (car.d < (2+4*lane+1)) &&  changing_lane) {
        changing_lane = false;
        cout << "- Lane transition completed!" << endl;
    }
    
    /** Sensor fusion information of closest vehicles ahead and behind by s value in all the lanes **/
    // Store the s value of closest cars to vehicle that are ahead. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_s_ahead = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Store the s value of closest cars to vehicle form behind in each lane. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_s_behind = {std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
    
    // Store the s value of closest cars to vehicle that are ahead. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_speed_ahead = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Store the s value of closest cars to vehicle form behind in each lane. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_speed_behind = {std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
    
    // Store the s value of closest cars to vehicle that are ahead. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_d_ahead = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    // Store the s value of closest cars to vehicle form behind in each lane. The indexes correspond to the respective lane 0:left, 1:center and 2:right
    vector<double> closest_d_behind = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

    // Set current lane as fastest lane
    int fastest_lane = lane;
    
    /** 1. SENSOR FUSION. Find the closest vehicles in every relevant direction **/
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

        /**
         * Detects closest cars to the autonomous vehicle and their d, speed values in each lane. We are interested in the closest cars 
         * ahead and behind the autonomous vehicle
         */ 
        if (abs(s-car.s)<120 && d > 0 && d < 12) {
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
    
    /** 2. DISTANCE LIMITS FLAGS. Finds whether there are vehicles too close ahead and behind **/
    int closest_ahead_index = lane;
    double temp = std::numeric_limits<double>::max();
    for (unsigned int j = 0; j< closest_d_ahead.size(); ++j)
    {
        if ((closest_d_ahead[j] > (4*lane-0.75)) && (closest_d_ahead[j]<(4.75+4*lane))) {
            if (temp > closest_s_ahead[j])
            {
                temp = closest_s_ahead[j];
                closest_ahead_index = j;
            }
        }
    }

    if (closest_ahead_index != lane) {
        cout << "  ! Vehicle coming to current lane from lane index: " << closest_ahead_index << endl;
    }

    // Minimum space gap required to keep a safe distance between autonomous vehicle and car ahead  
    double min_safe_distance_ahead = std::max((car.speed_ref*car.speed_ref - closest_speed_ahead[closest_ahead_index]*closest_speed_ahead[closest_ahead_index]),0.0)/(2*9.5)+5;

    // Car is ahead the autonomous vehicles is the distances is less than 20 Meters
    if ((closest_s_ahead[closest_ahead_index]-car.s) < min_safe_distance_ahead) {
        // If car is too close do some action    
        if (!too_close[1]) too_close[1] = true;
        //cout << "Close distance variables-> diff: " << (closest_s_ahead[lane]-car.s) << " , closest: " << closest_s_ahead[lane] << " , car: " << car.s << endl; 
    } else {
        if (too_close[1]) too_close[1] = false;
    }

    // Minimum space gap required to keep a safe distance between autonomous vehicle and car behind  
    double min_safe_distance_behind = std::max((closest_speed_behind[lane]*closest_speed_behind[lane] - car.speed_ref*car.speed_ref),0.0)/(2*9.5)+5;

    if ((car.s - closest_s_behind[lane]) < min_safe_distance_behind) {
        if (!too_close[0]) {
            too_close[0] = true;
            cout << "  ! Vehicle behind is too close." << endl;
        }
    } else {
        if (too_close[0]) {
            too_close[0] = false;
            cout << "  ! Vehicle is safe behind." << endl;
        }
    }

    /** 3. LANE SELECTION: Select next lane to transition or say in the same lane using future vehicle speed */
    /** Identify whether right or left lanes are free of obstacles in the upper and lower right/left directions.
     * This step is fundamental to then decide if a change of lane should be performed
     */ 

    // Lane transition possible directions: -1 to the left, +1 to the right
    vector<int> lane_transition = {-1 , 1};

    // Too close at left and right lanes flag initialization
    too_close_left = {false, false};
    too_close_right = {false, false};

    for (unsigned int i = 0; i < lane_transition.size(); ++i) {
        
        int temp_lane = lane + lane_transition[i];
        
        if (temp_lane >= 0 && temp_lane <=2) {
            
            
            // Minimum space gap required to keep a safe distance between autonomous vehicle and car behind  
            double min_safe_distance_behind = std::max((closest_speed_behind[temp_lane]*closest_speed_behind[temp_lane] - car.speed_ref*car.speed_ref),0.0)/(2*9.5)+5;
            // Blocked at lower direction. It also tests whether the relative speeds are enough to mantain minimum safe distance after 2 seconds.
            bool blocked_behind = !((car.s-closest_s_behind[temp_lane]+(car.speed_ref-closest_speed_behind[temp_lane]-9.5)*2.0) > min_safe_distance_behind);
            
            // Minimum space gap required to keep a safe distance between autonomous vehicle and car ahead  
            double min_safe_distance_ahead = std::max((car.speed_ref*car.speed_ref - closest_speed_ahead[temp_lane]*closest_speed_ahead[temp_lane]),0.0)/(2*9.5)+5;
            // Blocked at upper direction. It also tests whether the relative speeds are enough to mantain minimum safe distance after 2 seconds.
            bool blocked_ahead = !((closest_s_ahead[temp_lane]-car.s+(closest_speed_ahead[temp_lane]+9.5-car.speed_ref)*2.0) > min_safe_distance_ahead);

            if (blocked_behind) {
                if (lane_transition[i]==1) {
                    // Set availability as false for right lane transition in lower right direction
                    too_close_right[0] = true;
                } else {
                    // Set availability as false for left lane transition in lower left direction
                    too_close_left[0] = true;
                }
            }
            if (blocked_ahead) {
                if (lane_transition[i]==1) {
                    // Set availability as false for right lane transition in upper right direction
                    too_close_right[1] = true;
                } else {
                    // Set availability as false for left lane transition in upper left direction
                    too_close_left[1] = true;
                }   
            }      
        }
    }

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
    // Use fastest lane direction as the preferred lane transition direction. Speed must be larger than 30 MPH for the lane transition to be allowed
    if ((lane != fastest_lane) && car.speed > 30.0) {

        bool free_left = lane>0 && !too_close_left[1] && !too_close_left[0] && !changing_lane;
        bool free_right = lane<2 && !too_close_right[1] && !too_close_right[0] && !changing_lane;

        int prev_lane = lane;
        // If car ahead is too close consider changing lane.
        if (too_close[1]) {
            if (fastest_lane < lane) {
                // If current lane is center or right, left lane is free for lane change and changing_lane flag is false       
                if (free_left) {
                    lane -= 1;
                // Else, consider the right lane contraints
                } else if (free_right) {
                    lane +=1;
                }
            } else {
                // If current lane is center or left, right lane is free for lane change and changing_lane flag is false   
                if (free_right) {
                    lane += 1;
                // Else, consider the left lane contraints
                } else if (free_left) {
                    lane -= 1;
                } 
            }
        // Else, there are not obstacles ahead but the fastest lane is not current lane
        } else  {
            // Consider moving in the direction of the fast lane 
            if (fastest_lane < lane) {       
                if (free_left) {
                    lane -= 1;
                } 
            } else {
                if (free_right) {       
                    lane += 1;
                } 
            }
        }
        if (prev_lane!=lane) {
            if (prev_lane > lane) std::cout << "+ Fastest lane: " << fastest_lane << ", going Left." << std::endl;
            else std::cout << "+ Fastest lane: " << fastest_lane << ", going Right." << std::endl;
            changing_lane = true;
        }
    }

    /** 4. UPDATE SPEED BASED ON VEHICLE DISTANCE LIMITS FLAGS. **/
    /**
     * Behavior selection: Change speed.  If car ahead is too close reduce the speed, else increase speed
     */ 
    if ((too_close[1]) && car.speed_ref > 0)
    {
        // If car is 5 meters or less ahead reduce speed with an acceleration of 9.5m/s2
        if ((closest_s_ahead[closest_ahead_index] - car.s) < 5.0) {
            car.speed_ref += (-9.5*0.02);
        // Else reduce speed with an acceleration enough to match the speed of current target 5 Meters before collision 
        } else {
            double target_accel = ((closest_speed_ahead[closest_ahead_index]*closest_speed_ahead[closest_ahead_index] \
                                                    - car.speed_ref*car.speed_ref)/(2*(closest_s_ahead[closest_ahead_index]-car.s-5.0)));
            if (target_accel < -9.5) target_accel = -9.5;
            car.speed_ref += target_accel*0.02;
        }
    // If autonomous vehicle is not too close to cars ahead, speed up
    } else if (too_close[0] && car.speed_ref <= (49.5*1609.0/3600.0-0.19)) {
        // If car behind is too close increase speed with an acceleration of 9.5m/s2
        car.speed_ref += (9.5*0.02);
    } else if (car.speed_ref <= (49.5*1609.0/3600.0-0.10)) {
        car.speed_ref += (5.0*0.02);
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

    double step_s = 32.0;

    // Add evenly spaces points 30 m ahead of the starting point in frenet coordinates
    vector<double> next_wp0 = getXY(car.s+step_s*1, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); //28
    vector<double> next_wp1 = getXY(car.s+step_s*2, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car.s+step_s*3, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
    double target_x = step_s;
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
