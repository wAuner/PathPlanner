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
  // state variables
  //========================================================
  //========================================================
  double _car_x;
  double _car_y;
  double _car_s;
  double _d_coord;
  // angle in degrees
  double _car_yaw;
  // speed in mph
  double _car_speed;
  double _actual_distance;
  // ["sensor_fusion"] A 2d vector of cars and then that car's
  // [0: car's unique ID,
  // 1: car's x position in map coordinates,
  // 2: car's y position in map coordinates,
  // 3: car's x velocity in m/s,
  // 4: car's y velocity in m/s,
  // 5: car's s position in frenet coordinates,
  // 6: car's d position in frenet coordinates.
  vector<vector<double>> _sensor_fusion;
  bool _lane_conflict = false;
  int _curr_lane_idx;
  int _target_lane_idx;

  int _points2calc;
  int _conflict_idx;

  vector<double> _next_x_vals;
  vector<double> _next_y_vals;
  //========================================================



  // parameters
  //========================================================
  //========================================================
  double _safe_distance = 40;
  double _critical_distance = 30;
  // in mph, converted to m/s
  double _max_speed = 49.2 * 1609./3600;
  double _max_acc = 1.5;
  int _n_points_traj = 50;

  vector<int> _dist_vec {30, 90, 120};

  int _lane_change_dist_front = 10;
  int _lane_change_dist_back = -5;

  double _spline_start_x = 0;
  double _spline_start_y = 0;
  //========================================================




  // planning functions
  //========================================================
  //========================================================
  tk::spline InterpolateSpline(json &j, const vector<double> &map_waypoints_s,
                               const vector<double> &map_waypoints_x,
                               const vector<double> &map_waypoints_y);
  double AdaptSpeed(int);

  void ConsiderLaneChange();
  //========================================================



  // helpers
  //========================================================
  //========================================================
  int CheckLaneIdx(double);
  double Mph2Mps(double mph) { return mph * 1609. / 3600; }
  double ConvertSpeed2Increment(double speed, double target_x, double target_y);
  vector<double> Map2LocalTransform(double x_obs_glob, double y_obs_glob,
                                    double x_loc_ref, double y_loc_ref, double theta);

  vector<double> Local2MapTransform(double x_obs_loc, double y_obs_loc,
                                    double x_loc_ref, double y_loc_ref, double theta);
  double CalculateObstacleSpeed(const vector<double>& vehicle) { return sqrt(pow(vehicle[3],2) + pow(vehicle[4],2)); }
  //========================================================



public:
  // API
  //========================================================
  //========================================================
  PathPlanner() = default;

  void Localize(json &j, vector<vector<double>> &sensor_fusion);

  void CalculateTrajectory(json &j, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                           const vector<double> &map_waypoints_y);

  int CheckLaneConflict(int laneidx);

  vector<double> GetTrajectoryX() { return _next_x_vals; }
  vector<double> GetTrajectoryY() { return _next_y_vals; }
  //========================================================

};


#endif //PATH_PLANNING_PATHPLANNER_H
