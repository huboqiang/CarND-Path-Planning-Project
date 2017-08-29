//
//  cost_functions.hpp
//  Path_Planning
//
//  Created by HuBq on 2017/8/25.
//
//

#ifndef cost_functions_hpp
#define cost_functions_hpp

#include <stdio.h>
#include "Vehicle.hpp"
#include <map>
#include <vector>
#include "init_config.h"


const double COLLISION  = 1000000;
const double DANGER     = 100000;
const double REACH_GOAL = 100000;
const double COMFORT    = 10000;
const double EFFICIENCY = 100;

const double DESIRED_BUFFER = 1.5;
const int    PLANNING_HORIZON = 2;

double distance_from_goal_lane(
  Vehicle &vehicle, 
  std::vector<Snapshot> &trajectory, 
  std::map<int, std::vector<predsl>> &predictions, 
  TrajectoryData data);

double inefficiency_cost(
  Vehicle &vehicle, 
  std::vector<Snapshot> &trajectory, 
  std::map<int, std::vector<predsl>> &predictions, 
  TrajectoryData data);

double collision_cost(
  Vehicle &vehicle, 
  std::vector<Snapshot> &trajectory, 
  std::map<int, std::vector<predsl>> &predictions, 
  TrajectoryData data);

double buffer_cost(
  Vehicle &vehicle, 
  std::vector<Snapshot> &trajectory, 
  std::map<int, std::vector<predsl>> &predictions, 
  TrajectoryData data);

double change_lane_cost(
  Vehicle &vehicle, 
  std::vector<Snapshot> &trajectory, 
  std::map<int, std::vector<predsl>> &predictions, 
  TrajectoryData data);

double calculate_cost(
  Vehicle &vehicle,
  std::vector<Snapshot> &trajectory,
  std::map<int, std::vector<predsl>> &predictions);

TrajectoryData get_helper_data(
  Vehicle &vehicle, 
  std::vector<Snapshot> &trajectory,
  std::map<int, std::vector<predsl>> &predictions);

std::map<int, std::vector<predsl>> filter_predictions_by_lane(
  std::map<int, std::vector<predsl>> &predictions, 
  int lane);

bool check_collision(Snapshot &snapshot, double s_previous, double s_now);
    
#endif /* cost_functions_hpp */
