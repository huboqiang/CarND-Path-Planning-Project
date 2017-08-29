//
//  Road.cpp
//  Path_Planning
//
//  Created by HuBq on 2017/8/24.
//
//
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>
#include "json.hpp"
#include "Road.hpp"
#include "Vehicle.hpp"


using namespace std;

/*
    random [0,1], see http://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
 */
float get_random()
{
  static default_random_engine e;
  static uniform_real_distribution<> dis(0, 1); // rage 0 - 1
  return dis(e);
}

Road::Road(double speed_limit_, double traffic_density, vector<double> &lane_speeds_){
  num_lanes = lane_speeds_.size();
  lane_speeds = lane_speeds_;
  speed_limit = speed_limit_;
  density = traffic_density;
  camera_center = update_width / 2;
  vehicles_added = 0;
  
}

Road::~Road(){};


void Road::populate_traffic_using_sensor(const nlohmann::json &sensor_fusion){
  for (int i=0; i<sensor_fusion.size(); i++){
    int car_id = sensor_fusion[i][0];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double s_ = sensor_fusion[i][5];
    double d_ = sensor_fusion[i][6];
    int lane_ = d_ / 4;
    Vehicle vehicle(lane_, s_, check_speed, 0);
    vehicle.state = "CS";
    vehicles.insert(std::pair<int, Vehicle>(car_id, vehicle));
  }
}

void Road::update_traffic_using_sensor(const nlohmann::json &sensor_fusion){
  for (int i=0; i<sensor_fusion.size(); i++){
    int car_id = sensor_fusion[i][0];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double s_ = sensor_fusion[i][5];
    double d_ = sensor_fusion[i][6];
    int lane_ = d_ / 4;
    Vehicle v = vehicles.find(car_id)->second;
    //double prev_v = v.v;
    v.lane = lane_;
    v.s = s_;
    v.v = check_speed;
    //v.a = (check_speed-prev_v) / 0.05;
    vehicles.find(car_id)->second = v;
  }
}

void Road::add_ego(int lane_num, double s, init_config &config_data){
  Vehicle ego(lane_num, s, 0.0, 0.0);
  ego.configure(config_data);
  goal_lane = ego.goal_lane;
  goal_s = ego.goal_s;
  ego.state = "KL";
  vehicles.insert ( std::pair<int,Vehicle>(ego_key, ego) );
}

void Road::update_ego(int lane_num, double s_, double v_){
  Vehicle v = vehicles.find(ego_key)->second;
  double prev_v = v.v;
  v.lane = lane_num;
  v.s = s_;
  v.v = v_;
  v.a = (v_-prev_v);
  vehicles.find(ego_key)->second = v;
  //cout << "V and A is:" << v.v << "," << prev_v << "," << v.a << endl;
}

Vehicle Road::get_ego(){
  return vehicles.find(ego_key)->second;
}

bool Road::is_ego_init(){
  bool found = true;
  if(vehicles.find(ego_key) == vehicles.end() ){
    found = false;
  }
  return found;
}

void Road::advance(){
  map<int, vector<predsl>> predictions;
  for (map<int,Vehicle>::iterator it=vehicles.begin(); it!=vehicles.end(); ++it){
    int v_id = it->first;
    vector<predsl> preds = vehicles.find(v_id)->second.generate_predictions();
    predictions.insert ( std::pair<int,vector<predsl>>(v_id, preds) );
  }
  for (map<int,Vehicle>::iterator it=vehicles.begin(); it!=vehicles.end(); ++it){
    int v_id = it->first;
    Vehicle v = it->second;
    if(v_id == ego_key){
      v.update_state(predictions);
      //v.realize_state(predictions);
    }
    //v.increment();
    it->second = v;
  }
}






void Road::show_road(){
  double s_ = vehicles.find(ego_key)->second.s;
  camera_center = max(s_, update_width/2.0);
  double s_min = max(camera_center - update_width/2.0, 0.0);
  double s_max = s_min + update_width;
  vector<vector<string>> road;
  for(int i=0; i<update_width; i++){
    vector<string> road_tmp;
    for(int ln=0; ln<num_lanes; ln++){
      road_tmp.push_back("     ");
    }
    road.push_back(road_tmp);
  }
  
  if( (s_min <= goal_s) and (goal_s < s_max)){
    int idx = (int) (goal_s-s_min);
    road[idx][goal_lane] = " -G- ";
  }
  for (map<int,Vehicle>::iterator it=vehicles.begin(); it!=vehicles.end(); ++it){
    int v_id = it->first;
    Vehicle v = it->second;
    if( (s_min <= v.s) and (v.s < s_max)){
      string marker;
      if(v_id == ego_key){
        marker = ego_rep;
      }
      else{
        //cout << v_id << endl;
        stringstream ss;
        ss << setw(3)  << setfill('0') << v_id;
        //cout << "222" << endl;
        marker = " " + ss.str() + " ";
      }
      int idx = (int) (v.s-s_min);
      road[idx][v.lane] = marker;
    }
  }
  
  string s_out = "";
  int i = (int) s_min;
  for (int l=0; l<road.size(); l++){
    if (i % 20 == 0){
      stringstream ss;
      ss << setw(3)  << setfill('0') << i;
      string s_outtmp = " " + ss.str() + " ";
      s_out += s_outtmp;
    }
    else{
      s_out += "     ";
    }
    i += 1;
    string s_road = " " + road[l][0] + " ";
    for (int ll=1; ll<road[l].size(); ll++){
      s_road += "| " + road[l][ll] + " ";
    }
    s_out += "|" + s_road + "|" + "\n";
  }
  cout << s_out << endl;
}

