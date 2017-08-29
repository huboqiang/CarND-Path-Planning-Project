//
//  Vehicle.hpp
//  Path_Planning
//
//  Created by HuBq on 2017/8/24.
//
//

#ifndef Vehicle_hpp
#define Vehicle_hpp

#include <stdio.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <random>
#include "init_config.h"

class Vehicle{
public:
  Vehicle(int l, double s_, double v_, double a_);
  virtual ~Vehicle();
  std::vector<predsl> generate_predictions(int horizon=10);
  void update_state(std::map<int, std::vector<predsl>> &predictions);
  void realize_state(std::map<int, std::vector<predsl>> &predictions);
  void increment(double dt=1);
  void configure(init_config config_data);
  std::string _get_next_state(std::map<int, std::vector<predsl>> &predictions);
  std::vector<Snapshot> _trajectory_for_state(
    std::string &state,
    std::map<int, std::vector<predsl>> &predictions, int horizon=5);
  
  Snapshot snapshot();
  void restore_state_from_snapshot(Snapshot &snapshot_);
  
  
  Snapshot state_at(int t);
  void realize_constant_speed();
  void realize_keep_lane(std::map<int, std::vector<predsl>> &predictions);
  void realize_lane_change(std::map<int, std::vector<predsl>> &predictions, int direct);
  void realize_prep_lane_change(std::map<int, std::vector<predsl>> &predictions, int direct);
  double _max_accel_for_lane(std::map<int, std::vector<predsl>> &predictions, int lane, double s);
  
  int L = 1;
  int preferred_buffer = 6;
  
  int lane;
  double s;
  double v;
  double a;
  std::string state = "CS";
  
  double max_acceleration;
  double target_speed;
  int lanes_available;
  int goal_lane;
  double goal_s;
};
#endif /* Vehicle_hpp */
