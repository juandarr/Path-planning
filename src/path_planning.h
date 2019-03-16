/**
 * Path Planning class header
 * Implementation of behavior planning and trajectory generation
 * for the tour of an autonomous vechile in a high speed highway
 * 
 * Created on: March 15, 2019
 * Author: Juan Rios
 */

#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H

#include <vector>
#include <iostream>
#include "json.hpp"

using std::vector;
using nlohmann::json;

// Struct to store main localization information of car
struct Car {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double speed_ref;
};

class PathPlanning {
    public:
        // Contructor
        /**
         * Initializes PathPlanning class by defining the initial reference velocity,
         * current lane and changing lane state.
         * @param lane Current lane command to control car motion
         * @param changing_lane State of lane transition behavior 
         * @param ref_vel Reference velocity to control car motion in [m/s]
         */ 
        PathPlanning(): lane(1), changing_lane(false) {}

        // Destructor 
        ~PathPlanning() {}
        
        /**
         * behaviorSelection Performs behavior selection from a set of options [stay_in_lane, change_lane_right, 
         * change_lane_left, prepare_lange_change] according the sensor_fusion information about the car
         * surrounding, the car localization data and the size of the previous path vector
         * @para car Car localization information: x, y, s, d, yaw and speed
         * @param sensor_fusion Sensor fusion information of other vehicles detected in the environment
         * @param prev_size Size of previous path vector
         */ 
        void behaviorSelection(Car &car, json &sensor_fusion, int prev_size);
        
        /**
         * trajectoryGeneration Generates a smooth trajectory using the lane selection defined by the behaviorSelection 
         * algorithm. 
         * @param car Car localization information: x, y, s, d, yaw and speed
         * @param previous_path_x X coordinates oof previous path
         * @param previous_path_y Y coordinates of previous path
         * @param map_waypoints_s S coordinates of map waypoints (map reference)
         * @param map_waypoints_x X coordinates of map waypoints (map reference)
         * @param map_waypoints_y Y coordinates of map waypoints (map reference)
         * @output Return the optimal trajectory to follow considering contraints and fastest path
         */ 
        vector<vector<double>> trajectoryGeneration(Car &car, vector<double> previous_path_x, vector<double> previous_path_y, \
                        vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y);

    private:
        // Current lane directive for the controller
        int lane;
        // Lane change state. True when the car is changing lane. False when it is not or has completed transition
        bool changing_lane;
        
        
};

#endif // PATH_PLANNING_H_

