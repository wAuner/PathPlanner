//
// Created by mimisvm on 2/22/18.
//
#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "helper.h"
#include "json.hpp"
#include "spline.h"

using json = nlohmann::json;
using std::vector;

class PathPlanner
{
private:
  double car_x;
  double car_y;
  double car_s;
  double d_coord;
  double car_yaw;
  double car_speed;
  vector<vector<double>> sensor_fusion;

  double max_speed = 49.5;
  double max_acc = 0.15;

  bool lane_conflict = false;
  int curr_lane_idx;
  int n_points_traj = 50;
  int points2calc;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // spline
  tk::spline s;
  void interpolateSpline(json &j, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                         const vector<double> &map_waypoints_y);

  int check_lane_idx(double);

  double mph2mps(double mph) { return mph * 1609 / 3600; }

  vector<double> map2localTransform(double x_obs_glob, double y_obs_glob,
                                     double x_loc_ref, double y_loc_ref, double theta);

  vector<double> local2mapTransform(double x_obs_loc, double y_obs_loc,
                                    double x_loc_ref, double y_loc_ref, double theta);




public:
  PathPlanner() = default;

  void localize(json &j, vector<vector<double>> &sensor_fusion);

  // vector<vector<double>> predictions;
  void calculate_trajectory(json &j, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                              const vector<double> &map_waypoints_y);

  int checkLaneConflict();

  vector<double> getTrajectoryX();
  vector<double> getTrajectoryY();
};


#endif //PATH_PLANNING_PATHPLANNER_H
