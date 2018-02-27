//
// Created by mimisvm on 2/22/18.
//

#include "PathPlanner.h"
#include "spline.h"
#include <map>
#include <cmath>

void PathPlanner::localize(json &j, vector<vector<double>> &sensor_fusion)
{
  // Main car's localization Data
  this->car_x = j[1]["x"];
  this->car_y = j[1]["y"];
  this->car_s = j[1]["s"];
  this->d_coord = j[1]["d"];
  this->car_yaw = j[1]["yaw"];
  this->car_speed = j[1]["speed"];
  this->sensor_fusion = sensor_fusion;

  //std::cout << "speed: " << this->car_speed << std::endl;
  //std::cout << "yaw: " << this->car_yaw << std::endl;

  this->curr_lane_idx = check_lane_idx(d_coord);





}


void PathPlanner::calculate_trajectory(json &j, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                                       const vector<double> &map_waypoints_y)
{


  interpolateSpline(j, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  int conflict_idx = PathPlanner::checkLaneConflict();

  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];

  next_x_vals = previous_path_x;
  next_y_vals = previous_path_y;

  // Previous path's end s and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];

  for (int i=0; i < this->n_points_traj - previous_path_x.size(); ++i) {
    double dist_inc = 0.4;
    double s_new;
    double d_new;
    if (not previous_path_x.empty()) {
      s_new = end_path_s + dist_inc * (i + 1);
      d_new = 2 + 4 * this->curr_lane_idx;
    } else {
      s_new = this->car_s + dist_inc * (i + 1);
      d_new = 2 + 4 * this->curr_lane_idx;
    }
    vector<double> xy = getXY(s_new, d_new, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);

  }






  // TODO smooth trajectory using spline
  /*
  tk::spline s;
  s.set_points(next_x_vals, next_y_vals);
  next_y_vals.clear();
  for (double x : next_x_vals) {
    next_y_vals.push_back(s(x));
  }
   */
}


vector<double> PathPlanner::getTrajectoryX()
{
  return next_x_vals;
}


vector<double> PathPlanner::getTrajectoryY()
{
  return next_y_vals;
}

//checks whether a car is in the current lane and returns it's index, if no car returns -1
int PathPlanner::checkLaneConflict()
{
  // check if a car is within 50m in the same lane and return its index
  for (int caridx = 0; caridx < this->sensor_fusion.size(); ++caridx) {
    if (this->sensor_fusion[caridx][5] - car_s > 0 & this->sensor_fusion[caridx][5] - car_s < 50) {
      int lane_traffic = check_lane_idx(this->sensor_fusion[caridx][6]);
      if (lane_traffic == curr_lane_idx) {
        std::cout << "Lane conflict detected: caridx: " << caridx << " d value: " << sensor_fusion[caridx][6] << std::endl;
        std::cout << "s value: " << this->sensor_fusion[caridx][5] << std::endl;
        this->lane_conflict = true;
        return caridx;
      }
    }
  }
  this->lane_conflict = false;
  return -1;
}

int PathPlanner::check_lane_idx(double d_coord)
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
void PathPlanner::interpolateSpline(json &j, const vector<double> &map_waypoints_s,
                                    const vector<double> &map_waypoints_x,
                               const vector<double> &map_waypoints_y)
{
  // Previous path data given to the Planner
  // previous path is returned from simulator and in global coordinates
  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];

  this->points2calc = this->n_points_traj - previous_path_x.size();

  // spline interpolation
  vector<double> spline_x {0}, spline_y {0};

  unsigned long prev_size = previous_path_x.size();
  // push last two points of previous path on spline point list if existant
  if (not previous_path_x.empty()) {
    // if previous path exists, use last two points
    for (int i : {2, 1}) {
      vector<double> point = map2localTransform(previous_path_x[prev_size - i], previous_path_y[prev_size - i],
                                                this->car_x, this->car_y, deg2rad(this->car_yaw));
      spline_x.push_back(point[0]);
      spline_x.push_back(point[1]);
    }
  }
  // add points further away
  for (int dist : {30, 60, 90}) {
    int d = 2 + (this->curr_lane_idx * 4);
    double s = car_s + dist;
    vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> xy_trans = map2localTransform(xy[0], xy[1], this->car_x, this->car_y, deg2rad(this->car_yaw));
    spline_x.push_back(xy_trans[0]);
    spline_y.push_back(xy_trans[1]);
  }

  this->s.set_points(spline_x, spline_y);
}

// converts points from global to local (car) reference frame
vector<double> PathPlanner::map2localTransform(double x_obs_glob, double y_obs_glob,
                                         double x_loc_ref, double y_loc_ref, double theta)
{
  double shift_x = x_obs_glob - x_loc_ref;
  double shift_y = y_obs_glob - y_loc_ref;
  double x_transformed = shift_x * cos(-theta) - shift_y * sin(-theta);
  double y_transformed = shift_x * sin(-theta) + shift_y * cos(-theta);

  return {x_transformed, y_transformed};
}

vector<double> PathPlanner::local2mapTransform(double x_obs_loc, double y_obs_loc,
                                               double x_loc_ref, double y_loc_ref, double theta)
{
  double x_map = cos(theta) * x_obs_loc - sin(theta) * y_obs_loc + x_loc_ref;
  double y_map = sin(theta) * x_obs_loc + cos(theta) * y_obs_loc + y_loc_ref;

  return {x_map, y_map};
}





