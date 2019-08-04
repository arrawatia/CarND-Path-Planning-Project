#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

int main()
{
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
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

  // Car's lane. Starting at middle lane.
  int lane = 1;

  // Reference velocity.
  double reference_velocity = 0.0; // mph

  h.onMessage([&reference_velocity, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // This is a set of points that were not executed by the simulator during this
          // cycle. We want to add new points to this path so that it becomes 50 points long.
          int previous_path_size = previous_path_x.size();

          // Preventing collitions.
          if (previous_path_size > 0)
          {
            car_s = end_path_s;
          }

          // PREDICT STEP - Figure out where other cars on the road will be in future.
          bool is_too_close_to_car_ahead = false;
          bool is_left_lane_occupied = false;
          bool is_right_lane_occupied = false;
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float other_car_d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double other_car_speed = sqrt(vx * vx + vy * vy);
            double other_car_s = sensor_fusion[i][5];

            // Estimate other car's position in the future (Basically push the trajectory forward)
            other_car_s += ((double)previous_path_size * 0.02 * other_car_speed);

            // As lanes are 4m wide, we need to check +/- 2m from the center.
            // Find the lane the other car is in

            int other_car_lane = -1;
            if (other_car_d > 0 && other_car_d < 4)
            {
              other_car_lane = 0;
            }
            else if (other_car_d > 4 && other_car_d < 8)
            {
              other_car_lane = 1;
            }
            else if (other_car_d > 8 && other_car_d < 12)
            {
              other_car_lane = 2;
            }

            // The car is on the other side of the road, ignore it.

            if (other_car_lane < 0)
            {
              continue;
            }

            bool is_car_in_our_lane = (other_car_lane == lane);

            // If the car is in our lane, check if we need to take action.
            // Only take an action if the car is in front or less than 30 m ahead.
            if (is_car_in_our_lane)
            {

              bool is_car_in_front = other_car_s > car_s;
              bool is_class_less_than_30m_away = (other_car_s - car_s) < 30;
              is_too_close_to_car_ahead |= is_car_in_front && is_class_less_than_30m_away;
            }
            else
            {
              // Check if the other car is to the left or right.
              // Mark the lane as occupied if the other car is less than 30 m ahead or 30 m behind
              // LEFT Check
              if (other_car_lane - lane == -1)
              {
                is_left_lane_occupied |= car_s - 30 < other_car_s && car_s + 30 > other_car_s;
              }
              else // RIGHT Check
                  if (other_car_lane - lane == 1)
              {

                is_right_lane_occupied |= car_s - 30 < other_car_s && car_s + 30 > other_car_s;
              }
            }
          }

          // BEHAVIOR PLANNING - Change lanes if it safe or reduce speed.
          double velocity_change = 0;
          const double MAX_VELOCITY = 49.5;
          const double MAX_ACCELERATION = .224;
          if (is_too_close_to_car_ahead)
          { // Move to the left lane if not already in the left most lane and it is safe to do so.
            if (!is_left_lane_occupied && lane > 0)
            {
              lane--;
            }
            else // Move to the right lane if not already in the right most lane and it is safe to do so.
                if (!is_right_lane_occupied && lane != 2)
            {
              lane++;
            }
            else // No lane change possible, slow down in the current lane.
            {
              velocity_change -= MAX_ACCELERATION;
            }
          }
          else
          {
            if (lane != 1)
            { // If we are not on the center lane, try to get back to the center lane if it is safe to do so.
              if ((lane == 0 && !is_right_lane_occupied) || (lane == 2 && !is_left_lane_occupied))
              {
                lane = 1; // Back to center.
              }
            }
            // Speed up so that the car travels at the optimum velocity.
            if (reference_velocity < MAX_VELOCITY)
            {
              velocity_change += MAX_ACCELERATION;
            }
          }

          // Create a list of widely spaced (x,y) anchor waypoints, evenly spaced at 30 m
          // We will use these points as anchor points on the spline and then fill up the points
          // b/w them based on the desired speed.

          vector<double> anchor_waypoints_x;
          vector<double> anchor_waypoints_y;

          // Starting reference x, y and yaw states
          // This will either where the car is or the last point of the last path.
          // We will use this to change to the car's body frame and back.

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // The idea is to figure out the last couple of points in the previous path that
          // the car was following and calculating what angle the car was heading in using
          // these 2 points.
          // Add these points as the first 2 anchor points.
          // The previous path is almost empty use the car state as the reference state.

          if (previous_path_size < 2)
          {

            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            anchor_waypoints_x.push_back(prev_car_x);
            anchor_waypoints_y.push_back(prev_car_y);

            anchor_waypoints_x.push_back(car_x);
            anchor_waypoints_y.push_back(car_y);
          }
          else // Use the last point of the previous path as the anchor point
          {
            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];

            double prev_ref_x = previous_path_x[previous_path_size - 2];
            double prev_ref_y = previous_path_y[previous_path_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            anchor_waypoints_x.push_back(prev_ref_x);
            anchor_waypoints_y.push_back(prev_ref_y);

            anchor_waypoints_x.push_back(ref_x);
            anchor_waypoints_y.push_back(ref_y);
          }

          // std::cout << "anchor_waypoints_x = " << anchor_waypoints_x << std::endl;
          // std::cout << "anchor_waypoints_y = " << anchor_waypoints_y << std::endl;

          // Add 3 more anchor points. 30 m apart
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          anchor_waypoints_x.push_back(next_wp0[0]);
          anchor_waypoints_y.push_back(next_wp0[1]);

          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          anchor_waypoints_x.push_back(next_wp1[0]);
          anchor_waypoints_y.push_back(next_wp1[1]);

          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          anchor_waypoints_x.push_back(next_wp2[0]);
          anchor_waypoints_y.push_back(next_wp2[1]);

          // std::cout << "anchor_waypoints_x = " << anchor_waypoints_x << std::endl;
          // std::cout << "anchor_waypoints_y = " << anchor_waypoints_y << std::endl;

          // Making coordinates to local car coordinates.
          for (int i = 0; i < anchor_waypoints_x.size(); i++)
          {
            // std::cout << "anchor_waypoints_x = " << anchor_waypoints_x[i] << std::endl;
            // std::cout << "anchor_waypoints_y = " << anchor_waypoints_y[i] << std::endl;

            double shift_x = anchor_waypoints_x[i] - ref_x;
            double shift_y = anchor_waypoints_y[i] - ref_y;

            anchor_waypoints_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            anchor_waypoints_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

            // std::cout << "anchor_waypoints_x = " << anchor_waypoints_x[i] << std::endl;
            // std::cout << "anchor_waypoints_y = " << anchor_waypoints_y[i] << std::endl;
          }

          // Add the anchor points to the spline
          tk::spline spline;
          spline.set_points(anchor_waypoints_x, anchor_waypoints_y);

          // Plan the path
          vector<double> future_path_x;
          vector<double> future_path_y;

          // Add any remaining points from the previous path to the future path.
          // Helps out with the transistion b/w previous and future paths

          for (int i = 0; i < previous_path_size; i++)
          {
            future_path_x.push_back(previous_path_x[i]);
            future_path_y.push_back(previous_path_y[i]);
          }

          // Calculate how to break up the spline. Make sure the car travels at
          // the reference velocity
          // See diagram in walkthrough to understand this calculation
          // Create segments from the spline on 30 m ahead.
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_distance = sqrt(target_x * target_x + target_y * target_y);

          // Keep track of how far we come across. This tracks the end x values of last segment we
          // generated. Which is used as the starting point for the next segment
          double segment_start_x = 0;

          // Fill up the rest of the future path with points from the spline.
          int points_delta = 50 - previous_path_size;
          for (int i = 1; i < points_delta; i++)
          {
            reference_velocity += velocity_change;
            if (reference_velocity > MAX_VELOCITY)
            {
              reference_velocity = MAX_VELOCITY;
            }
            else if (reference_velocity < MAX_ACCELERATION)
            {
              reference_velocity = MAX_ACCELERATION;
            }

            double reference_velocity_m_per_s = reference_velocity / 2.24;
            // Number of spline segments required to maintain reference velocity
            double N = target_distance / (0.02 * reference_velocity_m_per_s);

            // Size of each spline segment
            double segment_size = target_x / N;

            double x_point = segment_start_x + segment_size;
            double y_point = spline(x_point);
            // std::cout << "spline(target_x) = " << x_point << " y=" << y_point << std::endl;

            // Move the segment starting point forward for the next iteration.
            segment_start_x = x_point;

            // Rotate back from the car's body frame to world frame
            double tx = x_point;
            double ty = y_point;

            x_point = ref_x + (tx * cos(ref_yaw) - ty * sin(ref_yaw));
            y_point = ref_y + (tx * sin(ref_yaw) + ty * cos(ref_yaw));

            // std::cout << "Rotate back x = " << x_point << " y=" << y_point << std::endl;

            future_path_x.push_back(x_point);
            future_path_y.push_back(y_point);
          }

          json msgJson;

          msgJson["next_x"] = future_path_x;
          msgJson["next_y"] = future_path_y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          // std::cout << "msg = " << msg << std::endl;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}