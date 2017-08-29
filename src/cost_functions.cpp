//
//  cost_functions.cpp
//  Path_Planning
//
//  Created by HuBq on 2017/8/25.
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

double distance_from_goal_lane(Vehicle &vehicle, vector<Snapshot> &trajectory, map<int, vector<predsl>> &predictions, TrajectoryData data){
  double distance = abs(data.end_lanes_from_goal);
  distance = max(distance, 1.0);
  double time_to_goal = distance / data.avg_speed;
  int lanes = data.end_lanes_from_goal;
  double multiplier = 5.0 * lanes / time_to_goal;
  double cost = multiplier * REACH_GOAL;
  return cost;
}

double inefficiency_cost(Vehicle &vehicle, vector<Snapshot> &trajectory, map<int, vector<predsl>> &predictions, TrajectoryData data){
  double speed = data.avg_speed;
  double target_speed = vehicle.target_speed;
  double diff = target_speed - speed;
  double pct = diff / target_speed;
  double multiplier = pct * pct;
  return multiplier * EFFICIENCY;
}

double collision_cost(Vehicle &vehicle, vector<Snapshot> &trajectory, map<int, vector<predsl>> &predictions, TrajectoryData data){
  double cost = 0.0;
  if(data.collides.is_collides){
    int time_til_collision = data.collides.at_pos;
    double exponent = (double) (time_til_collision * time_til_collision);
    double mult = exp(exponent);
    cost = mult*COLLISION;
  }
  return cost;
}

double buffer_cost(Vehicle &vehicle, vector<Snapshot> &trajectory, map<int, vector<predsl>> &predictions, TrajectoryData data){
  double closest = data.closest_approach;
  double cost = 0.0;
  if (closest == 0){
    cost = 10 * DANGER;
  }
  else{
    double timesteps_away = closest / data.avg_speed;
    if(timesteps_away > DESIRED_BUFFER){
      cost = 0.0;
    }
    else{
      double multiplier = 1.0 - pow((timesteps_away / DESIRED_BUFFER), 2.0);
      cost = multiplier * DANGER;
    }
  }
  return cost;
}

double change_lane_cost(Vehicle &vehicle, vector<Snapshot> &trajectory, map<int, vector<predsl>> &predictions, TrajectoryData data){
  int proposed_lanes = data.proposed_lane;
  int cur_lanes = trajectory[0].lane;
  double cost = 0.0;
  if(proposed_lanes > cur_lanes){
    cost = COMFORT;
  }
  if(proposed_lanes < cur_lanes){
    cost = -COMFORT;
  }
  if(cost != 0.0){
    cerr << "Cost for lane change is " << cost << endl;
  }
  return cost;
}


double calculate_cost(Vehicle &vehicle, vector<Snapshot> &trajectory, map<int, vector<predsl>> &predictions){
  TrajectoryData trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;
  cost += distance_from_goal_lane(vehicle, trajectory, predictions, trajectory_data);
  cost += inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
  cost += collision_cost(   vehicle, trajectory, predictions, trajectory_data);
  cost += buffer_cost(      vehicle, trajectory, predictions, trajectory_data);
  cost += change_lane_cost( vehicle, trajectory, predictions, trajectory_data);
  return cost;
}

TrajectoryData get_helper_data(Vehicle &vehicle,
                               vector<Snapshot> &trajectory,
                               map<int, vector<predsl>> &predictions){
  Snapshot current_snapshot = trajectory[0];
  Snapshot first            = trajectory[1];
  Snapshot last             = trajectory[trajectory.size()-1];
  
  double end_distance_to_goal = vehicle.goal_s - last.s;
  double end_lanes_from_goal  = abs(vehicle.goal_lane - last.lane);
  double dt = (double) trajectory.size();
  int proposed_lane = first.lane;
  double avg_speed = (last.s - current_snapshot.s) / dt;
  
  vector<double> accels;
  double closest_approach = numeric_limits<double>::max();
  collides_info collides = {false, -1000};
  Snapshot last_snap = trajectory[0];
  map<int, vector<predsl>> filtered = filter_predictions_by_lane(predictions, proposed_lane);
  for(int i=1; i<PLANNING_HORIZON+1; i++){
    accels.push_back(trajectory[i].a);
    for (map<int,vector<predsl>>::iterator it=filtered.begin(); it!=filtered.end(); ++it){
      vector<predsl> v = it->second;
      predsl state = v[i];
      predsl last_state = v[i-1];
      bool vehicle_collides = check_collision(trajectory[i], last_state.s, state.s);
      if(vehicle_collides){
        collides.is_collides = true;
        collides.at_pos = i;
      }
      double dist = (double) (state.s - trajectory[i].s);
      if(dist < closest_approach){
        closest_approach = dist;
      }
    }
    last_snap = trajectory[i];
  }
  
  double max_accel = numeric_limits<double>::min();
  vector<double> rms_accels;
  double rms_acceels_sum = 0;
  for(int i=0; i<accels.size(); i++){
    rms_accels.push_back(accels[i]*accels[i]);
    rms_acceels_sum += accels[i]*accels[i];
    if(accels[i] > max_accel){
      max_accel = accels[i];
    }
  }
  
  int num_accels = rms_accels.size();
  double rms_acceleration = rms_acceels_sum / (double) num_accels;
  return {proposed_lane, avg_speed, max_accel, rms_acceleration, closest_approach, end_distance_to_goal, end_lanes_from_goal, collides};
}

map<int, vector<predsl>> filter_predictions_by_lane(map<int, vector<predsl>> &predictions, int lane){
  map<int, vector<predsl>> filtered;
  for (map<int,vector<predsl>>::iterator it=predictions.begin(); it!=predictions.end(); ++it){
    int v_id = it->first;
    vector<predsl> predicted_traj = it->second;
    if(predicted_traj[0].lane == lane and v_id != -1){
      filtered.insert ( pair<int,vector<predsl>>(v_id, predicted_traj) );
    }
  }
  return filtered;
}

bool check_collision(Snapshot &snapshot, double s_previous, double s_now){
  bool is_collision = false;
  double s = snapshot.s;
  double v = snapshot.v;
  double v_target = s_now - s_previous;
  if (s_previous < s){
    if (s_now >= s){
      is_collision = true;
    }
    else{
      is_collision = false;
    }
  }
  if (s_previous > s){
    if (s_now <= s){
      is_collision = true;
    }
    else{
      is_collision = false;
    }
  }
  if (s_previous == s){
    if (v_target >= v){
      is_collision = false;
    }
    else{
      is_collision = true;
    }
  }
  return is_collision;
}
