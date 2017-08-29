//
//  init_config.h
//  Path_Planning
//
//  Created by HuBq on 2017/8/25.
//
//

#ifndef init_config_h
#define init_config_h

struct predsl {
  double s;
  int lane;
};

struct init_config {
  double speed_limit;
  int num_lane;
  predsl goal;
  double max_acceleration;
};

struct Snapshot{
  int lane;
  double s;
  double v;
  double a;
  std::string state;
};

struct collides_info{
  bool is_collides;
  int at_pos;
};

struct TrajectoryData{
  int proposed_lane;
  double avg_speed;
  double max_acceleration;
  double rms_acceleration;
  double closest_approach;
  double end_distance_to_goal;
  double end_lanes_from_goal;
  collides_info collides;
};



#endif /* init_config_h */
