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

  //std::cout << "speed: " << car_speed << std::endl;
  //std::cout << "yaw: " << car_yaw << std::endl;

  _curr_lane_idx = CheckLaneIdx(_d_coord);
  _target_lane_idx = CheckLaneIdx(j[1]["end_path_d"]);
}


void PathPlanner::CalculateTrajectory(json &j, const vector<double> &map_waypoints_s,
                                      const vector<double> &map_waypoints_x,
                                      const vector<double> &map_waypoints_y)
{
  _conflict_idx = PathPlanner::CheckLaneConflict(_curr_lane_idx);

  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];

  _next_x_vals = previous_path_x;
  _next_y_vals = previous_path_y;

  /*
  // Previous path's end s and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];
   */

  unsigned long prev_size = previous_path_x.size();

  double last_x_loc = 0;

  // check if there are points from previous paths and if so, use them as starting point
  if (not previous_path_x.empty()) {
    // convert last point from global to local reference frame
    last_x_loc = (previous_path_x.back() - _car_x) * cos(-deg2rad(_car_yaw))
                        - (previous_path_y.back() - _car_y) * sin(-deg2rad(_car_yaw));
  }

  // TODO: fix acceleration issue

  double ref_vel = AdaptSpeed(_conflict_idx);
  tk::spline s = InterpolateSpline(j, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  double increment = ConvertSpeed2Increment(ref_vel, 30, s(30));
  for (int i=0; i < _points2calc; ++i) {

    double x = last_x_loc + increment * (i + 1);
    double y = s(x);

    vector<double> xy_glob = Local2MapTransform(x, y, _car_x, _car_y, deg2rad(_car_yaw));

    _next_x_vals.push_back(xy_glob[0]);
    _next_y_vals.push_back(xy_glob[1]);
  }
  //std::cout << "\n\n";

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
        //std::cout << "Lane conflict detected: caridx: " << caridx << ", d value: "
        //          << sensor_fusion[caridx][6] << std::endl;
        //std::cout << "s value: " << sensor_fusion[caridx][5] << std::endl;

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
        //std::cout << "car dist: " << dist << std::endl;
        //std::cout << "nearest car: " << nearest_car << std::endl;
        //std::cout << "my car s: " << car_s << std::endl;
        //std::cout << "obstacle s: " << obstacles[caridx][5] << std::endl << std::endl;
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

  _points2calc = _n_points_traj - previous_path_x.size();

  // spline interpolation
  vector<double> spline_x {_spline_start_x}, spline_y {_spline_start_y};

  unsigned long prev_size = previous_path_x.size();
  // push last two points of previous path on spline point list if existant
  if (not previous_path_x.empty()) {
    // if previous path exists, use last two points
    for (int i : {2, 1}) {
      vector<double> point = Map2LocalTransform(previous_path_x[prev_size - i], previous_path_y[prev_size - i],
                                                _car_x, _car_y, deg2rad(_car_yaw));
      spline_x.push_back(point[0]);
      spline_y.push_back(point[1]);
    }
  }
  // add points further away
  for (int dist : _dist_vec) {
    int d = 2 + (_target_lane_idx * 4);
    double s = _car_s + dist;
    vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> xy_trans = Map2LocalTransform(xy[0], xy[1], _car_x, _car_y, deg2rad(_car_yaw));
    spline_x.push_back(xy_trans[0]);
    spline_y.push_back(xy_trans[1]);
  }
  tk::spline s;

  s.set_points(spline_x, spline_y);
  return s;
}

// converts points from global to local (car) reference frame
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

double PathPlanner::AdaptSpeed(int conflict_idx)
{
  double ref_vel = Mph2Mps(_car_speed);
  if (not _lane_conflict || _actual_distance > _safe_distance) {
    ref_vel += ref_vel + _max_acc > _max_speed ? _max_speed - ref_vel : _max_acc;
  } else if (_lane_conflict && _actual_distance <= _safe_distance) {
    ref_vel = CalculateObstacleSpeed(_sensor_fusion[conflict_idx]);
    ConsiderLaneChange();
  } else if (_lane_conflict && _actual_distance <= _critical_distance) {
    ref_vel -= _max_acc;
    ConsiderLaneChange();
    //std::cout << "following vehilce " << _conflict_idx <<" with speed "<< vel_obstacle <<"\n";
  }
  return ref_vel;
}

// speed in m/s
double PathPlanner::ConvertSpeed2Increment(double speed, double target_x, double target_y)
{
  double dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double N = dist / (speed * 0.02);

  return target_x / N;
}


void PathPlanner::ConsiderLaneChange()
{
  vector<int> lanesToCheck;
  if (_curr_lane_idx == 0 || _curr_lane_idx == 2) {
    lanesToCheck.push_back(1);
  } else if (_curr_lane_idx == 1) {
    lanesToCheck.push_back(0);
    lanesToCheck.push_back(2);
  }

  for (int lane : lanesToCheck) {
    bool lane_good = true;
    for (vector<double> vehicle : _sensor_fusion) {
      int obstacle_lane = CheckLaneIdx(vehicle[6]);
      double& obstacle_s = vehicle[5];
      double obstacle_vel =CalculateObstacleSpeed(vehicle);
      double dist = obstacle_s - _car_s;

      // check if car in that lane is in front of us and slower
      if (obstacle_lane == lane && obstacle_vel < Mph2Mps(_car_speed) &&
          dist > _lane_change_dist_back) {
        lane_good = false;
        break;
        // check if car is in that lane and faster but with not enough space to pull out
      } else if (obstacle_lane == lane && obstacle_vel > Mph2Mps(_car_speed) &&
          (dist > _lane_change_dist_back && dist < _lane_change_dist_front)) {
        lane_good = false;
        break;
      }
    }
    if (lane_good) {
      _target_lane_idx = lane;
      std::cout << "Changing lane to " << lane << std::endl;
    }
  }
  return;
}






