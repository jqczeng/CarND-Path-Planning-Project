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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

string car_change_state(int next_lane, string car_state, double car_d) {

    double goal_next_lane_lb = 2 + 4*next_lane - 2;
    double goal_next_lane_rb = 2 + 4*next_lane + 2;

    cout << "car_d: " << car_d << " goal_lane_lb: " << goal_next_lane_lb << " goal_lane_rb: " << goal_next_lane_rb << endl;
    if (car_d > goal_next_lane_lb + 1 && car_d < goal_next_lane_rb - 1) {
      cout << "Finished changing lane" << endl;
      car_state = "KL";
    } else if (car_state == "PL") {
      cout << "car changing left" << endl;
    } else if (car_state == "PR") {
      cout << "car changing right" << endl;
    }

  return car_state;
}

bool detect_vehicles_on_lane(int next_lane, double car_s, vector<vector<double>> sensor_fusion, int prev_size) {
  bool vehicles_detected = false; 
  int buffer_dist_ahead = 30.0;
  int buffer_dist_behind = 10.0;

    // find ref_v to use
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane 
    float d = sensor_fusion[i][6];
    
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_car_s = sensor_fusion[i][5];
    double check_speed = sqrt(vx*vx + vy*vy);

    double next_lane_lb = 2 + 4*next_lane - 2;
    double next_lane_rb = 2 + 4*next_lane + 2;

    check_car_s += ((double) prev_size * 0.02 * check_speed);

    // detect the vehicle in the next lane
    if (d < next_lane_rb && d > next_lane_lb) {
      if ((check_car_s  > car_s) && (abs(check_car_s - car_s) < buffer_dist_ahead)) {
        cout << "Vehicle ahead with: " << abs(check_car_s - car_s) << " m dist" << endl;
        vehicles_detected = true;
        break;
      } else if ((check_car_s < car_s) && (abs(check_car_s - car_s) < buffer_dist_behind)) {
        cout << "Vehicle behind with: " << abs(check_car_s - car_s) << " m dist" << endl;
        vehicles_detected = true;
        break;
      }
    }
  }

  return vehicles_detected;
}

int lane_selection(int lane, double car_s, vector<vector<double>> sensor_fusion, int prev_size) {
  
  int goal_lane = lane;

  cout << "too close and changing lanes!" << endl;

  if (lane == 1) {
    bool left_lane_full = detect_vehicles_on_lane(0, car_s, sensor_fusion, prev_size);
    bool right_lane_full = detect_vehicles_on_lane(2, car_s, sensor_fusion, prev_size);

    if (!left_lane_full && right_lane_full)
    {
      cout << "No cars on left, changing left" << endl;
      goal_lane = 0;
    } else if (left_lane_full && !right_lane_full) {
      cout << "No cars on right, changing right" << endl;
      goal_lane = 2;
    }
  }
  else if (lane == 0) {
    bool right_lane_full = detect_vehicles_on_lane(1, car_s, sensor_fusion, prev_size);

    if (!right_lane_full) {
      cout << "changing lane right" << endl;
      goal_lane = 1;
    }
  }
  else if (lane == 2) {
    bool left_lane_full = detect_vehicles_on_lane(1, car_s, sensor_fusion, prev_size);

    if (!left_lane_full) {
      cout << "changing lane left" << endl;
      goal_lane = 1;
    }
  }

  return goal_lane;
}



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


  int lane = 1; // starting lane

  double ref_vel = 0.0; // mph

  string car_state = "KL";

  int next_lane;

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

  h.onMessage([&ref_vel, &lane, &car_state, &next_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          // Prediction of other cars 
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;

          cout << "Currently on lane: " << lane << " car_d: " << car_d << " car state: " << car_state << endl;

          // find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // car is in my lane 
            float d = sensor_fusion[i][6];

            float cur_lane_rbound = 2 + 4*lane + 2;
            float cur_lane_lbound = 2 + 4*lane - 2;
            double check_car_s = sensor_fusion[i][5];

            if (d < cur_lane_rbound && d > cur_lane_lbound) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              
              // predict the other car's current s value
              check_car_s += ((double) prev_size * 0.02 * check_speed);

              // the car ahead of us is too close
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                too_close = true;
              }
            } 
          }

          // // if too close to the car ahead, change lanes 
          if (too_close) {
            if (car_state == "KL")
            {
              next_lane = lane_selection(lane, car_s, sensor_fusion, prev_size);
              
              // prepare change lane right
              if (next_lane - lane > 0) {
                car_state = "PLCR";
              } 
              // prepare change lane left
              else if (next_lane - lane < 0) {
                car_state = "PLCL";
              } 
              else {
                ref_vel -= 0.224;
              }
            }
          } else if (!too_close && ref_vel < 49.5) {
            ref_vel += 0.224;
          }

          if (car_state == "PLCL") {
              bool lane_full = detect_vehicles_on_lane(next_lane, car_s, sensor_fusion, prev_size);
              if (!lane_full) {
                cout << "lane: " << next_lane << " clear. Pass left" << endl;
                lane = next_lane;
                car_state = "PL";
              } else {
                car_state = "KL";
              }
          }
          else if (car_state == "PLCR") {
              bool lane_full = detect_vehicles_on_lane(next_lane, car_s, sensor_fusion, prev_size);
              if (!lane_full) {
                cout << "lane: " << next_lane << " clear. Pass right" << endl;
                lane = next_lane;
                car_state = "PR";
              } else {
                car_state = "KL";
              }
          }
          else if (car_state == "PL" || car_state == "PR") {
            cout << "passing to lane: " << lane << endl;
            car_state = car_change_state(lane, car_state, car_d);
          }

          // Trajectory 
          vector<double> ptsx; 
          vector<double> ptsy; 

          // reference x, y and yaw states
          // later we will interpolate these waypoints with a spline and fill it with more points that control speed
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if the previous size is almost empty, use the car as startng reference
          if (prev_size < 2) {
            // use two points that make the path tangent to the car 
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          // use the previous path's end point as starting reference 
          else {
            // redefine reference state as previous path end point 
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // use two points that make the path tangent to the previous path's end point 
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // in Frenet add evenly 30m spaced points ahead of the starting reference 
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Making coordinates to local car coordinates.
          for ( int i = 0; i < ptsx.size(); i++ ) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create a spline 
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // define the actual (x,y) points to use for the planner 
          vector<double> next_x_vals;
          vector<double> next_y_vals;

           // Output path points from previous path for continuity.
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // start with all of the previous path points from last time 
          // Calculate distance y position on 30 m ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for( int i = 1; i < 50 - prev_size; i++ ) {

            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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