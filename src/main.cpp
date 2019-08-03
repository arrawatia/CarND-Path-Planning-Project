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

  // start in lane 1
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 49.5;

  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
      std::cout << "data = " << data << std::endl;

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

          int previous_path_size = previous_path_x.size();

          // Create a list of widely spaced (x,y) anchor waypoints, evenly spaced at 30 m
          // We will use these points as anchor points on the spline and then fill up the points
          // b/w them based on the desired speed.
          vector<double> anchor_waypoints_x;
          vector<double> anchor_waypoints_y;

          // Starting reference x, y and yaw states
          // This will either where the car is or the last point of the last path.
          // We will use this to change to the car's body frame and back.
          double reference_state_x;
          double reference_state_y;
          double reference_state_yaw;

          // The idea is to figure out the last couple of points in the previous path that
          // the car was following and calculating what angle the car was heading in using 
          // these 2 points.
          
          // Add these points as the first 2 anchor points.

          // The previous path is almost empty use the car state as the reference state.
          if(previous_path_size < 2) 
          {
            reference_state_x = car_x;
            reference_state_y = car_y;
            reference_state_yaw = deg2rad(car_yaw);

            // Use the 2 points to make the path which is tangential to the car
            // Go back in time to generate a point behind the current one.
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            anchor_waypoints_x.push_back(prev_car_x);
            anchor_waypoints_y.push_back(prev_car_y);
            
            anchor_waypoints_x.push_back(car_x);
            anchor_waypoints_y.push_back(car_y);
          }
          else // Use the last point of the previous path as the anchor point
          {
            reference_state_x = previous_path_x[previous_path_size - 1];
            reference_state_y = previous_path_y[previous_path_size - 1];
            
            // 
            double prev_reference_state_x = previous_path_x[previous_path_size - 2];
            double prev_reference_state_y = previous_path_y[previous_path_size - 2];

            reference_state_yaw = atan2(reference_state_y - prev_reference_state_x,  reference_state_y - prev_reference_state_y);

            // Two points that make the path tangential to previous path's last point.
            anchor_waypoints_x.push_back(prev_reference_state_x);
            anchor_waypoints_y.push_back(prev_reference_state_y);
            
            anchor_waypoints_x.push_back(reference_state_x);
            anchor_waypoints_y.push_back(reference_state_y);
          }

          // Add 3 more anchor points. 30 m apart
          for (int i = 1; i <= 3; i++) {
            vector<double> next_waypoint = getXY(car_s + (i * 30), (2 + 4 * (lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            anchor_waypoints_x.push_back(next_waypoint[0]);
            anchor_waypoints_y.push_back(next_waypoint[1]);
          }

          // Convert the anchor points to the car's body frame so that the 
          // last point on the path becomes the origin or (0,0) and the yaw 
          // becomes zero.

          for (int j = 0; j <= anchor_waypoints_x.size(); j++) {
            // Transform
            double shift_x = anchor_waypoints_x[j] - reference_state_x;
            double shift_y = anchor_waypoints_y[j] - reference_state_y;

            anchor_waypoints_x[j] = shift_x * cos(0 - reference_state_yaw) - shift_y * sin(0 - reference_state_yaw);
            anchor_waypoints_y[j] = shift_x * sin(0 - reference_state_yaw) + shift_y * cos(0 - reference_state_yaw);
          }

          // Add the anchor points to the spline
          tk::spline spline;
          spline.set_points(anchor_waypoints_x, anchor_waypoints_y);

          // Plan the path
          vector<double> future_path_x;
          vector<double> future_path_y;

          // Add any remaining points from the previous path to the future path. 
          // Helps out with the transistion b/w previous and future paths
          for (int k = 0; k <= previous_path_x.size(); k++) {
            future_path_x.push_back(previous_path_x[k]);
            future_path_y.push_back(previous_path_y[k]);
          }

          // Calculate how to break up the spline. Make sure the car travels at
          // the reference velocity
          // See diagram in walkthrough to understand this calculation
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_distance = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          // Fill up the rest of the future path with points from the spline.

          int points_delta = 50 - previous_path_size;
          for (int l = 1; l <= points_delta; l++) {
            // See video for calculation details
            double N = target_distance / (0.2 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = spline(x_point);

            // Rotate back from the car's body frame to world frame
            double reference_x = x_point;
            double reference_y = y_point;

            x_point = reference_x * cos(reference_state_yaw) - reference_y * sin(reference_state_yaw);
            y_point = reference_x * sin(reference_state_yaw) + reference_y * cos(reference_state_yaw);

            x_point += reference_x;
            y_point += reference_y;

            future_path_x.push_back(x_point);
            future_path_y.push_back(y_point);
          
          }

          json msgJson;

          
          msgJson["next_x"] = future_path_x;
          msgJson["next_y"] = future_path_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          //  std::cout << "msg = " << msg << std::endl;

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