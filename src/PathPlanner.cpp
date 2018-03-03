//
// Created by mimisvm on 2/22/18.
//

#include "PathPlanner.h"
#include "spline.h"
#include <map>
#include <cmath>

void PathPlanner::Localize(json &j, vector<vector<double>> &sensor_fusion)
{
  // Main car's localization Data
  _car_x = j[1]["x"];
  _car_y = j[1]["y"];
  _car_s = j[1]["s"];
  _d_coord = j[1]["d"];
  _car_yaw = j[1]["yaw"];
  _car_speed = j[1]["speed"];
  _sensor_fusion = sensor_fusion;
  _curr_lane_idx = CheckLaneIdx(_d_coord);
  _target_lane_idx = CheckLaneIdx(j[1]["end_path_d"]);
}

// calculates the trajectory and calls all necessary member functions, except Localize
void PathPlanner::CalculateTrajectory(json &j, const vector<double> &map_waypoints_s,
                                      const vector<double> &map_waypoints_x,
                                      const vector<double> &map_waypoints_y)
{
  _conflict_idx = PathPlanner::CheckLaneConflict(_curr_lane_idx);

  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];

  _next_x_vals = previous_path_x;
  _next_y_vals = previous_path_y;

  _ref_vel = AdaptSpeed(_conflict_idx);
  tk::spline s = InterpolateSpline(j, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  double increment_approx = ConvertSpeed2Increment(_ref_vel, 30, s(30));

  double last_x_loc = 0;
  // check if there are points from previous paths and if so, use them as starting point
  if (previous_path_x.size() > 2) {
    // convert last point from global to local reference frame
    last_x_loc = (previous_path_x.back() - _x_ref) * cos(-_ref_yaw)
                 - (previous_path_y.back() - _y_ref) * sin(-_ref_yaw);
  }
  _points2calc = _n_points_traj - previous_path_x.size();
  for (int i=0; i < _points2calc; ++i) {

    double x_loc = last_x_loc + increment_approx * (i + 1);
    double y_loc = s(x_loc);
    vector<double> xy_glob = Local2MapTransform(x_loc, y_loc, _x_ref, _y_ref, _ref_yaw);

    _next_x_vals.push_back(xy_glob[0]);
    _next_y_vals.push_back(xy_glob[1]);
  }


}


//checks whether a car is in the current lane and returns it's index, if no car returns -1
int PathPlanner::CheckLaneConflict(int laneidx)
{
  vector<vector<double>> obstacles;
  // check if a car is within 50m in the same lane and return its index
  for (int caridx = 0; caridx < _sensor_fusion.size(); ++caridx) {
     if (_sensor_fusion[caridx][5] - _car_s > 0 &&
         _sensor_fusion[caridx][5] - _car_s < _safe_distance) {
      int lane_traffic = CheckLaneIdx(_sensor_fusion[caridx][6]);
      if (lane_traffic == laneidx) {
        obstacles.push_back(_sensor_fusion[caridx]);
      }
      _lane_conflict = laneidx == _curr_lane_idx ? true : false;
    }
  }

  // find nearest car in that lane
  if (not obstacles.empty()) {

    double dist = 1000;
    int nearest_car;
    for (int caridx=0; caridx < obstacles.size(); ++caridx) {
      if (obstacles[caridx][5] - _car_s < dist) {
        dist = obstacles[caridx][5] - _car_s;
        nearest_car = static_cast<int>(obstacles[caridx][0]);
      }
    }
    _actual_distance = dist;
    return nearest_car;
  }
  _lane_conflict = false;
  return -1;
}

int PathPlanner::CheckLaneIdx(double d_coord)
{
  // check lane
  if (d_coord < 4) {
    return 0;
  } else if (d_coord > 4 & d_coord <= 8) {
    return 1;
  } else if (d_coord > 8 & d_coord <= 12) {
    return 2;
  }
}

// performs a spline interpolation for the car's local reference frame
tk::spline PathPlanner::InterpolateSpline(json &j, const vector<double> &map_waypoints_s,
                                          const vector<double> &map_waypoints_x,
                                          const vector<double> &map_waypoints_y)
{
  // Previous path data given to the Planner
  // previous path is returned from simulator and in global coordinates
  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];

  vector<double> spline_x , spline_y ;

  unsigned long prev_size = previous_path_x.size();
  // push last two points of previous path on spline point list if existent
  _ref_yaw = deg2rad(_car_yaw);
  _x_ref = _car_x;
  _y_ref = _car_y;
  // this implementation assumes the reference frame at the tip of the current trajectory
  if (prev_size > 2) {
    double x1 = previous_path_x[prev_size - 1];
    double x0 = previous_path_x[prev_size - 2];
    double y1 = previous_path_y[prev_size - 1];
    double y0 = previous_path_y[prev_size - 2];

    spline_x.push_back(x0);
    spline_x.push_back(x1);

    spline_y.push_back(y0);
    spline_y.push_back(y1);

    _ref_yaw = atan2(y1 - y0, x1 - x0);
    _x_ref = x1;
    _y_ref = y1;

  } else {
    spline_x.push_back(_car_x - cos(_ref_yaw));
    spline_x.push_back(_car_x);

    spline_y.push_back(_car_y - sin(_ref_yaw));
    spline_y.push_back(_car_y);
  }
  // add points further away
  for (int dist : _dist_vec) {
    int d = 2 + (_target_lane_idx * 4);
    // optional adaptation for higher speeds to smoothen trajectory
    //dist += _ref_vel > 15 ? 20 : 0;
    double s = _car_s + dist;
    vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    spline_x.push_back(xy[0]);
    spline_y.push_back(xy[1]);
  }

  // convert points from global to local frame at the tip of the current path
  for (int i = 0; i < spline_x.size(); ++i) {
    vector<double> xy_trans = Map2LocalTransform(spline_x[i], spline_y[i], _x_ref, _y_ref, _ref_yaw);
    spline_x[i] = xy_trans[0];
    spline_y[i] = xy_trans[1];
  }

  tk::spline s;

  s.set_points(spline_x, spline_y);
  return s;
}

// converts points from global to local (car/trajectory) reference frame
vector<double> PathPlanner::Map2LocalTransform(double x_obs_glob, double y_obs_glob,
                                               double x_loc_ref, double y_loc_ref, double theta)
{
  double shift_x = x_obs_glob - x_loc_ref;
  double shift_y = y_obs_glob - y_loc_ref;
  double x_transformed = shift_x * cos(-theta) - shift_y * sin(-theta);
  double y_transformed = shift_x * sin(-theta) + shift_y * cos(-theta);

  return {x_transformed, y_transformed};
}

vector<double> PathPlanner::Local2MapTransform(double x_obs_loc, double y_obs_loc,
                                               double x_loc_ref, double y_loc_ref, double theta)
{
  double x_map = cos(theta) * x_obs_loc - sin(theta) * y_obs_loc + x_loc_ref;
  double y_map = sin(theta) * x_obs_loc + cos(theta) * y_obs_loc + y_loc_ref;

  return {x_map, y_map};
}

// handles speed control and initializes the search for a better lane if necessary
// returns speed in m/s, conflict idx is result of CheckLaneConflict
double PathPlanner::AdaptSpeed(int conflict_idx)
{
  double ref_vel = Mph2Mps(_car_speed);
  if (not _lane_conflict || _actual_distance > _safe_distance) {
    ref_vel += ref_vel + _max_acc > _max_speed ? _max_speed - ref_vel : _max_acc;
  } else if (_lane_conflict && _actual_distance <= _safe_distance) {
    if (ref_vel > CalculateObstacleSpeed(_sensor_fusion[conflict_idx])) {
      ref_vel -= _max_acc;
    }
    ConsiderLaneChange();
  } else if (_lane_conflict && _actual_distance <= _critical_distance) {
    ref_vel -= _max_acc;
    ConsiderLaneChange();
  }
  return ref_vel;
}

// converts the speed target in a position update with 20ms update rate
// approximates the spline linearly
// speed in m/s
double PathPlanner::ConvertSpeed2Increment(double speed, double target_x, double target_y)
{
  double dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double N = dist / (speed * 0.02);
  
  return target_x / N;
}

// examines the lane choices, calculates a cost for each option and chooses the lane
// w/ lowest cost
void PathPlanner::ConsiderLaneChange()
{
  vector<int> lanes_to_check;
  if (_curr_lane_idx == 0 || _curr_lane_idx == 2) {
    lanes_to_check.push_back(1);
  } else if (_curr_lane_idx == 1) {
    lanes_to_check.push_back(0);
    lanes_to_check.push_back(2);
  }

  // calculate costs for staying in lane
  double cost_curr_lane = _max_speed - Mph2Mps(_car_speed) + 1 / _actual_distance;
  double lowest_cost = cost_curr_lane;

  vector<int> lanes_w_cost;
  for (int lane : lanes_to_check) {
    bool lane_good = true;

    double nearest_dist = 10e4;
    double lane_speed = _max_speed;

    for (vector<double> vehicle : _sensor_fusion) {
      int obstacle_lane = CheckLaneIdx(vehicle[6]);
      double& obstacle_s = vehicle[5];
      double obstacle_vel = CalculateObstacleSpeed(vehicle);
      double dist = obstacle_s - _car_s;

      // check if car in that lane is in front of us and slower
      if (obstacle_lane == lane && obstacle_vel < Mph2Mps(_car_speed) &&
          (dist > _lane_change_dist_back && dist < 50)) {
        lane_good = false;
        break;
        // check if car is in that lane and faster but with not enough space to pull out
      } else if (obstacle_lane == lane && obstacle_vel > Mph2Mps(_car_speed) &&
          (dist > - (obstacle_vel * 0.5) && dist < _lane_change_dist_front)) {
        lane_good = false;
        break;
        // if no knockout criteria is met, identify nearest car for following cost calc
      } else if (obstacle_lane == lane && dist > 0 && dist < nearest_dist) {
        nearest_dist = dist;
        if (dist < 60) {
          lane_speed = obstacle_vel;
        }

      }
    }
    if (lane_good) {
      double cost_lane_change = _max_speed - lane_speed + _cost_for_lane_change
                                + 1 / nearest_dist;
      if (cost_lane_change < lowest_cost) {
        lowest_cost = cost_lane_change;
        _target_lane_idx = lane;
        std::cout << "Change lane to " << lane << std::endl;
      }
    }
  }
  return;
}

double PathPlanner::CalculateWaypointDistance(const vector<double> &prev_x, const vector<double> &prev_y)
{
  int vector_size = prev_x.size();
  const double& x0 = prev_x[vector_size - 2];
  const double& x1 = prev_x[vector_size - 1];
  const double& y0 = prev_x[vector_size - 2];
  const double& y1 = prev_x[vector_size - 1];
  double dist = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));

  return dist;
}






