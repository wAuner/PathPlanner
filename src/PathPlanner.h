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
  // angle in degrees
  double car_yaw;
  // speed in mph
  double car_speed;

  double safe_distance = 40;
  double critical_distance = 30;
  double actual_distance;
  // ["sensor_fusion"] A 2d vector of cars and then that car's
  // [car's unique ID, car's x position in map coordinates, car's y position in map coordinates,
  // car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates,
  // car's d position in frenet coordinates.
  vector<vector<double>> sensor_fusion;

  // in mph
  double max_speed = 49.5 * 1609./3600;
  double max_acc = 2;


  bool lane_conflict = false;
  int curr_lane_idx;
  int target_lane_idx;
  int n_points_traj = 50;
  int points2calc;
  int conflict_idx;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // spline
  //tk::spline s;
  tk::spline interpolateSpline(json &j, const vector<double> &map_waypoints_s,
                               const vector<double> &map_waypoints_x,
                               const vector<double> &map_waypoints_y);

  int check_lane_idx(double);

  double mph2mps(double mph) { return mph * 1609. / 3600; }

  vector<double> map2localTransform(double x_obs_glob, double y_obs_glob,
                                     double x_loc_ref, double y_loc_ref, double theta);

  vector<double> local2mapTransform(double x_obs_loc, double y_obs_loc,
                                    double x_loc_ref, double y_loc_ref, double theta);

  double adaptSpeed(int);

  double convertSpeed2Increment(double speed, double target_x, double target_y);

  void considerLaneChange();




public:
  PathPlanner() = default;

  void localize(json &j, vector<vector<double>> &sensor_fusion);

  // vector<vector<double>> predictions;
  void calculate_trajectory(json &j, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                              const vector<double> &map_waypoints_y);

  int checkLaneConflict(int laneidx);

  vector<double> getTrajectoryX();
  vector<double> getTrajectoryY();
};


#endif //PATH_PLANNING_PATHPLANNER_H
