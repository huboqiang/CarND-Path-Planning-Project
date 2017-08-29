//
//  Vehicle.cpp
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
#include <limits>
#include <random>
#include "Vehicle.hpp"
#include "init_config.h"
#include "cost_functions.hpp"

using namespace std;

Vehicle::Vehicle(int l, double s_, double v_, double a_){
  lane = l;
  s = s_;
  v = v_;
  a = a_;
}
Vehicle::~Vehicle(){}


vector<predsl> Vehicle::generate_predictions(int horizon){
  vector<predsl> predictions;
  for(int i=0; i<horizon; i++){
    Snapshot st_ = state_at(i);
    predsl sl_ = {st_.s, st_.lane};
    predictions.push_back(sl_);
  }
  return predictions;
}


/*
 Updates the "state" of the vehicle by assigning one of the
 following values to 'self.state':
 
 "KL" - Keep Lane
  - The vehicle will attempt to drive its target speed, unless there is
    traffic in front of it, in which case it will slow down.
 
 "LCL" or "LCR" - Lane Change Left / Right
  - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
    behavior for the "KL" state in the new lane.
 
 "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
 - The vehicle will find the nearest vehicle in the adjacent lane which is
   BEHIND itself and will adjust speed to try to get behind that vehicle.
 
 INPUTS
 - predictions
  A dictionary. The keys are ids of other vehicles and the values are arrays
  where each entry corresponds to the vehicle's predicted location at the
  corresponding timestep. The FIRST element in the array gives the vehicle's
  current position. Example (showing a car with id 3 moving at 2 m/s):
 
 {
  3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
  ]
 }
 */
void Vehicle::update_state(map<int, vector<predsl>> &predictions){
  state = _get_next_state(predictions);
}

string Vehicle::_get_next_state(map<int, vector<predsl>> &predictions){
  vector<string> states = {"KL", "LCL", "LCR"};
  //cout << lane << "," << lanes_available << endl;
  if(lane == 0){
    states.erase(states.begin()+1);  // remove LCL
  }
  if(lane == lanes_available-1){
    states.erase(states.begin()+2);  // remove LCR
  }
  
  double min_cost = numeric_limits<double>::max();
  string best_state;
  for(int i=0; i<states.size(); i++){
    map<int, vector<predsl>> predictions_copy = predictions;
    vector<Snapshot> trajectory = _trajectory_for_state(states[i], predictions_copy);
    double cost = calculate_cost(*this, trajectory, predictions_copy);
    if(cost < min_cost){
      min_cost = cost;
      best_state = states[i];
    }
  }
  
  return best_state;
}

vector<Snapshot> Vehicle::_trajectory_for_state(string &state_, map<int, vector<predsl>> &predictions, int horizon){
  Snapshot snapshot_ = snapshot();
  state = state_;
  vector<Snapshot> trajectory = {snapshot_};
  for(int i=0; i<horizon; i++){
    restore_state_from_snapshot(snapshot_);
    state = state_;
    realize_state(predictions);
    increment();
    trajectory.push_back(snapshot());
    for (map<int,vector<predsl>>::iterator it=predictions.begin(); it!=predictions.end(); ++it){
      vector<predsl> pp = it->second;
      pp.pop_back();
      it->second = pp;
    }
  }
  restore_state_from_snapshot(snapshot_);
  return trajectory;
}


void Vehicle::increment(double dt){
  s = s + v*dt;
  v = v + a*dt;
}

void Vehicle::configure(init_config config_data){
  target_speed = config_data.speed_limit;
  lanes_available = config_data.num_lane;
  max_acceleration = config_data.max_acceleration;
  predsl goal = config_data.goal;
  goal_lane = goal.lane;
  goal_s = goal.s;
}

Snapshot Vehicle::snapshot(){
  return Snapshot{lane, s, v, a, state};
}

void Vehicle::restore_state_from_snapshot(Snapshot &snapshot_){
  Snapshot st = snapshot_;
  lane = st.lane;
  s = st.s;
  v = st.v;
  a = st.a;
  state = st.state;
}


Snapshot Vehicle::state_at(int t){
  Snapshot st_;
  double s_ = s + v * t + 0.5 * a * t * t;
  double v_ = v + a * t;
  st_ = {lane, s_, v_, a, ""};
  return st_;
}

void Vehicle::realize_state(map<int, std::vector<predsl>> &predictions){
  if(state == "CS"){
    realize_constant_speed();
  }
  else if(state == "KL"){
    realize_keep_lane(predictions);
  }
  else if(state == "LCL"){
    realize_lane_change(predictions, -1);
  }
  else if(state == "LCR"){
    realize_lane_change(predictions, 1);
  }
  else if(state == "PLCL"){
    realize_prep_lane_change(predictions, -1);
  }
  else if(state == "PLCR"){
    realize_prep_lane_change(predictions, 1);
  }
}

void Vehicle::realize_constant_speed(){
  a = 0;
}
void Vehicle::realize_keep_lane(map<int, std::vector<predsl>> &predictions){
  a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_lane_change(map<int, std::vector<predsl>> &predictions, int direct){
  lane += direct;
  a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int, std::vector<predsl>> &predictions, int direct){

}

double Vehicle::_max_accel_for_lane(map<int, vector<predsl>> &predictions, int lane_, double s_){
  double delta_v_til_target = target_speed - v;
  double max_acc = min(max_acceleration, delta_v_til_target);
  vector<vector<predsl>> in_front;
  
  for (map<int,vector<predsl>>::iterator it=predictions.begin(); it!=predictions.end(); ++it){
    vector<predsl> predv = it->second;
    if(predv[0].lane == lane_ and predv[0].s > s_){
      in_front.push_back(predv);
    }
  }
  for (map<int,vector<predsl>>::iterator it=predictions.begin(); it!=predictions.end(); ++it){
    vector<predsl> predv = it->second;
    if(predv[0].lane == lane_ and predv[0].s > s_){
      in_front.push_back(predv);
    }
  }
  if(in_front.size() > 0){
    double delta_s_min = numeric_limits<double>::max();
    vector<predsl> leading;
    for(int i=0; i<in_front.size(); i++){
      double delta_s = in_front[i][0].s - s_;
      if(delta_s < delta_s_min){
        delta_s_min = delta_s;
        leading = in_front[i];
      }
    }
    double next_pos = leading[1].s;
    double my_next  = s_ + v;
    double separation_next = next_pos - my_next;
    double available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }
  return max_acc;
}
