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
#include "Road.hpp"
#include "Vehicle.hpp"
#include "cost_functions.hpp"
#include "utils.hpp"
#include "init_config.h"

using namespace std;

//constexpr double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180; }
//double rad2deg(double x) { return x * 180 / pi(); }


// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

bool check_new_lane_dist(json &sensor_fusion, double car_s, int lane_, int prev_size, bool is_ahead=true){
  bool too_close;
  for (int i=0; i<sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];
    if (d < (2 + 4 * lane_ + 2) and d > (2 + 4 * lane_ - 2)){
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];
      
      check_car_s += ((double)prev_size*0.02*check_speed);
      if(is_ahead){
        if( (check_car_s > car_s) and ((check_car_s-car_s) < 30)){
          too_close = true;
        }
      }
      else{
        if( (check_car_s < car_s) and ((car_s-check_car_s) < 20)){
          too_close = true;
        }
      }
      
    }
  }
  return too_close;
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
  string map_file_ = "../data/highway_map.csv"; // "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  loadHighWayMap(map_file_, map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
  
  
  double speed_limit = 49.5;
  vector<double> lane_speeds = {45.0, 40.0, 35.0};
  double traffic_density = 0.15;
  double max_accel = 9.0;
  predsl goal = {max_s, 1};
  Road road(speed_limit, traffic_density, lane_speeds);
  init_config ego_config = {speed_limit, static_cast<int>(lane_speeds.size()), goal, max_accel};

  ofstream outfile;
  string file_out = "./log";
  outfile.open(file_out.c_str());
  
  int lane = 1;
  double ref_vel = 0.0;
  
  int timeIdx = 0;
  int turn_lane_response_time = 200;
  int turn_lane_response_used = 0;
  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane, &timeIdx, &road, &ego_config, &turn_lane_response_time, &turn_lane_response_used, &outfile](
       uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    timeIdx += 1;
    string action = "KL";
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          json sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;
          
          if(not road.is_ego_init()){
            road.populate_traffic_using_sensor(sensor_fusion);
            road.add_ego(1, car_s, ego_config);
          }
          else{
            road.update_traffic_using_sensor(sensor_fusion);
            road.update_ego(lane, car_s, car_speed);
          }
          road.advance();
          Vehicle ego = road.get_ego();
          
          cout << "a=" << ego.a << endl;
          int prev_size = previous_path_x.size();
          if(turn_lane_response_used >0){
            turn_lane_response_used += 1;
            if(turn_lane_response_used == turn_lane_response_time){
              turn_lane_response_used = 0;
            }
          }
          else if( (ego.state != "KL") and (car_speed < 40.0) and (car_speed > 25) and (ego.a > 0)){
            if(lane > 0 and ego.state == "LCL"){
              bool new_lane_too_close_ahead  = check_new_lane_dist(sensor_fusion, car_s, lane-1, prev_size, true);
              bool new_lane_too_close_behind = check_new_lane_dist(sensor_fusion, car_s, lane-1, prev_size, false);
              cout << lane << "," << ego.state << "," << turn_lane_response_used << new_lane_too_close_ahead << "," << new_lane_too_close_behind << endl;
              if ((not new_lane_too_close_ahead) and (not new_lane_too_close_behind)){
                lane -= 1;
                turn_lane_response_used += 1;
                action = "LCL";
              }
              
            }
            if(lane < road.lane_speeds.size()-1 and ego.state == "LCR"){
              bool new_lane_too_close_ahead = check_new_lane_dist(sensor_fusion, car_s, lane+1, prev_size, true);
              bool new_lane_too_close_behind = check_new_lane_dist(sensor_fusion, car_s, lane+1, prev_size, false);
              cout << lane << "," << ego.state << "," << turn_lane_response_used << new_lane_too_close_ahead << "," << new_lane_too_close_behind << endl;
              if ((not new_lane_too_close_ahead) and (not new_lane_too_close_behind)){
                lane += 1;
                turn_lane_response_used += 1;
                action = "LCR";
              }
            }
          }
          
          // V2, run on lane and smooth. Need to learn how to pulls up with other cars
          // How to speed up with proper acc rate
          
          if(prev_size > 0){
            car_s = end_path_s;
          }
          bool too_close = check_new_lane_dist(sensor_fusion, car_s, lane, prev_size, true);
          if(too_close){
            ref_vel -= 0.224;
          }
          else if (ref_vel < 49.5){
            ref_vel += 0.224;
          }
          
          
          // get frenet in d=[last_point, current, +30, +60, +90] as ptsx, ptsy
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
        
          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          vector<double> next_wp0 = getXY(car_s+30, (4*lane+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (4*lane+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (4*lane+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // rotate ptsx, ptsy
          for (int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }
          
          // ptsx,ptsy to next_val. using (pre_len) prev_path points + (50-pre_len) prev_
          tk::spline s;
          s.set_points(ptsx, ptsy);
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_d = sqrt(target_x*target_x+target_y*target_y);
          double x_add_on = 0.0;
          
          double N = (target_d/(0.02*ref_vel/2.24));
          
          for(int i = 1; i <= 50-previous_path_x.size(); i++ ){
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw) + ref_x;
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw) + ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto vec_x = msgJson["next_x"];
          auto vec_y = msgJson["next_y"];
          
          auto vec_px = ptsx;
          auto vec_py = ptsy;
          
          outfile << car_s  << "," << car_d << "," << car_speed << "," << timeIdx << "," << car_x << "," <<  car_y << endl;
          outfile << sensor_fusion << endl;
          outfile << "[" << previous_path_x << "," << previous_path_y << "]" << endl;
          outfile << "[" << vec_x << "," << vec_y << "]" << endl;
          outfile << "[[" << vec_px[0] << "," << vec_px[1] << "," << vec_px[2] << "," << vec_px[3] << "," << vec_px[4] << "],[" << vec_py[0] << "," << vec_py[1] << "," << vec_py[2] << "," << vec_py[3] << "," << vec_py[4] << "]]" << endl;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
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
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
