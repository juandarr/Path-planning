#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <limits>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  // Define lane of current trajectory
  int lane = 1;
          
  double ref_vel = 0.0;

  bool changing_lane = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane, &ref_vel, &changing_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size(); 
          

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          if ((car_d < (2+4*lane+1)) && (car_d > (2+4*lane-1))) {
            changing_lane = false;
          }

          bool too_close = false;
          bool left_clear_up = true;
          bool left_clear_down = true;
          bool right_clear_up = true;
          bool right_clear_down = true;
          

          double transition_vel = ref_vel;
          double closest_car_up = std::numeric_limits<double>::max();
          double closest_car_left_down = std::numeric_limits<double>::min();
          double closest_car_left_up = std::numeric_limits<double>::max();
          double closest_car_right_down = std::numeric_limits<double>::min();
          double closest_car_right_up = std::numeric_limits<double>::max();

          double furthest_car_s = std::numeric_limits<double>::min();
          double fastest_lane = lane;
          vector<bool> empty_lanes = {true, true, true};

          //find ref_vel to use
          for (unsigned int i = 0; i < sensor_fusion.size(); ++i) {
            
            float d = sensor_fusion[i][6];

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += check_speed*0.02*prev_size;

            double check_speed_ms = check_speed*1609.0/3600.0;

            if (car_s < check_car_s && (check_car_s-car_s)<100 && d > 0 && d < 12) {
              empty_lanes[floor(d/4)] = false;
              if (check_car_s > furthest_car_s) {
                furthest_car_s = check_car_s;
                if (d < 4 && d >= 0) {
                  fastest_lane = 0;          
                } else if (d >= 4 && d <8) {
                  fastest_lane = 1;
                } else if (d >= 8 && d <= 12) {
                  fastest_lane = 2;
                }
              } 
            }  
            // Current lane
            if (d < (2+4*lane+3) && d > (2+4*lane-3)) {
              
              if ((car_s < check_car_s) && (check_car_s-car_s)< 30) {

                if (closest_car_up > check_car_s) {
                  closest_car_up = check_car_s;
                  // If car is too close do some action      
                  too_close = true;
                  transition_vel = check_speed_ms;
                }
              }
              // Right lane
            } else if  (d > (2+4*lane+2) && d < (2+4*lane+6)) {
              
              bool condition_prev_cars = ((car_s > check_car_s) && ((car_s+ref_vel*2)  < (check_car_s+check_speed_ms*2+35)));
              bool condition_ahead_cars = ((car_s < check_car_s) && (check_car_s-car_s) < 30 && (check_speed_ms < ref_vel*1.2));  
              
              if (condition_prev_cars) {
                if (closest_car_right_down < check_car_s) {
                  // Clear right lane for lane transition
                  right_clear_down = false;
                  closest_car_right_down = check_car_s;
                }
              }
              if (condition_ahead_cars) {
                if (closest_car_right_up > check_car_s) {
                  // Clear right lane for lane transition
                  right_clear_up = false;
                  closest_car_right_up = check_car_s;
                }
              }
              // Left lane
            } else if  (d < (2+4*lane-2) && d > (2+4*lane-6)) {
              
              bool condition_prev_cars = ((car_s > check_car_s) && ((car_s+ref_vel*2)  < (check_car_s+check_speed_ms*2+35)));
              bool condition_ahead_cars = ((car_s < check_car_s) && (check_car_s-car_s) < 30) && (check_speed_ms < ref_vel*1.2);
              
              if (condition_prev_cars) {
                if (closest_car_left_down < check_car_s) {
                  // Clear left lane for lane transition
                  left_clear_down = false;
                  closest_car_left_down = check_car_s;
                }
              }
              if (condition_ahead_cars) {
                if (closest_car_left_up > check_car_s) {
                  // Clear left lane for lane transition
                  left_clear_up = false;
                  closest_car_left_up = check_car_s;
                }
              }
            }
          }

          if (too_close)
          {
            if (abs(closest_car_up - car_s) < 5) {
              ref_vel -= 0.16;
            } else {
              ref_vel -= ((ref_vel - transition_vel)*0.02/4.0);
            }
          } else if (ref_vel < 22.12375) {
            ref_vel += 0.10;
          }

          if (empty_lanes[lane]==true) {
            fastest_lane = lane;
          } else {
            for (unsigned int i = 0; i < empty_lanes.size(); ++i) {
                if (empty_lanes[i]==true) {
                  fastest_lane = i;
                  break;
                }
            }
          } 
          
          if (too_close || lane != fastest_lane) {
            if (fastest_lane < lane) {       
                if (lane>0 && left_clear_up && left_clear_down && !changing_lane) {
                  lane -= 1;
                  changing_lane = true;
                } else if (lane<2 && right_clear_up && right_clear_down && !changing_lane) {
                  lane += 1;
                  changing_lane = true;
                }
              } else {
                if (lane<2 && right_clear_up && right_clear_down && !changing_lane) {
                  lane += 1;
                  changing_lane = true;
                } else if (lane>0 && left_clear_up && left_clear_down && !changing_lane) {
                  lane -= 1;
                  changing_lane = true;
                } 
              }
            }




          // Create a list of widely spaced (x,y) waypoints, evenly spaces at 30 m
          // We will interpolate this points using the spline method
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x, y and yaw states
          // We will reference the current car position or we will access the last path end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If the previous path is almost empty, use the car as starting reference

          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);  

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          // Use previous path points as starting reference
          else {
             
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2((ref_y-ref_y_prev),(ref_x-ref_x_prev));

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // Add evenly spaces points 30 m ahead of the starting point in frenet coordinates
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
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
          

          double N = target_distance / (0.02*ref_vel);
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
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //Follow the current lane
          /*
          double dist_inc = 0.469;
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s+(i+1)*dist_inc;
            double next_d = 6;

            vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals.push_back(next_xy[0]);
            next_y_vals.push_back(next_xy[1]);
          }*/

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}