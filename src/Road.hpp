//
//  Road.hpp
//  Path_Planning
//
//  Created by HuBq on 2017/8/24.
//
//

#ifndef Road_hpp
#define Road_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include "Vehicle.hpp"
#include "init_config.h"
#include "json.hpp"


class Road{
public:
  Road(double speed_limit_, double traffic_density, std::vector<double>&lane_speeds);
  virtual ~Road();

  void populate_traffic_using_sensor(const nlohmann::json &sensor_fusion);
  void update_traffic_using_sensor(const nlohmann::json &sensor_fusion);

  void add_ego(int lane_num, double s, init_config &config_data);
  void update_ego(int lane_num, double s_, double v_);
  Vehicle get_ego();
  bool is_ego_init();

  void advance();
  void show_road();
  double update_width = 70;
  std::string ego_rep = " *** ";
  int ego_key=-1;
  int num_lanes;
  std::vector<double> lane_speeds;
  std::map<int, Vehicle> vehicles;
  double speed_limit;
  double density;
  double camera_center;

  int vehicles_added;
  int goal_lane;
  double goal_s;
};


#endif /* Road_hpp */
