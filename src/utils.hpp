//
//  utils.hpp
//  Path_Planning
//
//  Created by HuBq on 2017/8/24.
//
//

#ifndef utils_hpp
#define utils_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include "json.hpp"

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y,
                    std::vector<double> &maps_x,
                    std::vector<double> &maps_y);


int NextWaypoint(double x, double y, double theta,
                 std::vector<double> &maps_x,
                 std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta,
                              std::vector<double> &maps_x,
                              std::vector<double> &maps_y);

std::vector<double> getXY(double s, double d,
                          std::vector<double> &maps_s,
                          std::vector<double> &maps_x,
                          std::vector<double> &maps_y);

void loadHighWayMap(const std::string &map_file_,
                    std::vector<double> &map_waypoints_x,
                    std::vector<double> &map_waypoints_y,
                    std::vector<double> &map_waypoints_s,
                    std::vector<double> &map_waypoints_dx,
                    std::vector<double> &map_waypoints_dy);

#endif /* utils_hpp */
