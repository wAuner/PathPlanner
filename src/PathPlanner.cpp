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
  this->target_lane_idx = check_lane_idx(j[1]["end_path_d"]);
}


void PathPlanner::calculate_trajectory(json &j, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                                       const vector<double> &map_waypoints_y)
{
  conflict_idx = PathPlanner::checkLaneConflict(curr_lane_idx);

  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];

  next_x_vals = previous_path_x;
  next_y_vals = previous_path_y;

  /*
  // Previous path's end s and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];
   */

  unsigned long prev_size = previous_path_x.size();

  double last_x_loc = 0;

  // check if there are points from previous paths and if so, use them as starting point
  if (not previous_path_x.empty()) {
    last_x_loc = (previous_path_x.back() - car_x) * cos(-deg2rad(car_yaw))
                        - (previous_path_y.back() - car_y) * sin(-deg2rad(car_yaw));
  }
  //std::cout << "points to calc: " << points2calc << " size prev. path: " << prev_size << "\n";
  // TODO: fix acceleration issue

  double ref_vel = adaptSpeed(conflict_idx);
  tk::spline s = interpolateSpline(j, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  double increment = convertSpeed2Increment(ref_vel, 30, s(30));
  for (int i=0; i < this->points2calc; ++i) {

    double x = last_x_loc + increment * (i + 1);
    double y = s(x);
    //std::cout << "x/y local: " << x << " / " << y <<" "<<"\n";

    vector<double> xy_glob = local2mapTransform(x, y, car_x, car_y, deg2rad(car_yaw));
    //std::cout << "x/y global: " << xy_glob[0] << " / " << xy_glob[1] <<" "<<"\n\n";

    next_x_vals.push_back(xy_glob[0]);
    next_y_vals.push_back(xy_glob[1]);
  }
  //std::cout << "\n\n";

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
int PathPlanner::checkLaneConflict(int laneidx)
{
  vector<vector<double>> obstacles;
  // check if a car is within 50m in the same lane and return its index
  for (int caridx = 0; caridx < this->sensor_fusion.size(); ++caridx) {
     if (this->sensor_fusion[caridx][5] - car_s > 0 &&
         this->sensor_fusion[caridx][5] - car_s < safe_distance) {
      int lane_traffic = check_lane_idx(this->sensor_fusion[caridx][6]);
      if (lane_traffic == laneidx) {
        //std::cout << "Lane conflict detected: caridx: " << caridx << ", d value: "
        //          << sensor_fusion[caridx][6] << std::endl;
        //std::cout << "s value: " << this->sensor_fusion[caridx][5] << std::endl;

        obstacles.push_back(this->sensor_fusion[caridx]);
      }
      this->lane_conflict = laneidx == curr_lane_idx ? true : false;
    }
  }

  // find nearest car in that lane
  if (not obstacles.empty()) {

    double dist = 1000;
    double nearest_car;
    for (int caridx=0; caridx < obstacles.size(); ++caridx) {
      if (obstacles[caridx][5] - car_s < dist) {
        dist = obstacles[caridx][5] - car_s;
        nearest_car = obstacles[caridx][0];
        //std::cout << "car dist: " << dist << std::endl;
        //std::cout << "nearest car: " << nearest_car << std::endl;
        //std::cout << "my car s: " << car_s << std::endl;
        //std::cout << "obstacle s: " << obstacles[caridx][5] << std::endl << std::endl;
      }
    }
    //target_lane_idx = 0;
    actual_distance = dist;
    return nearest_car;
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
tk::spline PathPlanner::interpolateSpline(json &j, const vector<double> &map_waypoints_s,
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

  vector<int> dist_vec {30, 60, 90};

  unsigned long prev_size = previous_path_x.size();
  // push last two points of previous path on spline point list if existant
  if (not previous_path_x.empty()) {
    // if previous path exists, use last two points
    for (int i : {2, 1}) {
      vector<double> point = map2localTransform(previous_path_x[prev_size - i], previous_path_y[prev_size - i],
                                                this->car_x, this->car_y, deg2rad(this->car_yaw));
      spline_x.push_back(point[0]);
      spline_y.push_back(point[1]);
    }
  }
  // add points further away
  for (int dist : dist_vec) {
    int d = 2 + (this->target_lane_idx * 4);
    double s = car_s + dist;
    vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> xy_trans = map2localTransform(xy[0], xy[1], this->car_x, this->car_y, deg2rad(this->car_yaw));
    spline_x.push_back(xy_trans[0]);
    spline_y.push_back(xy_trans[1]);
  }
  tk::spline s;

  s.set_points(spline_x, spline_y);
  return s;
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

double PathPlanner::adaptSpeed(int conflict_idx)
{
  double ref_vel = mph2mps(car_speed);
  if (not lane_conflict || actual_distance > safe_distance) {
    ref_vel += ref_vel + max_acc > max_speed ? max_speed - ref_vel : max_acc;
  } else if (lane_conflict && actual_distance <= safe_distance) {
    double& v_x_obst = sensor_fusion[conflict_idx][3];
    double& v_y_obst = sensor_fusion[conflict_idx][4];
    double vel_obstacle = sqrt(pow(v_x_obst,2) + pow(v_y_obst,2));
    ref_vel = vel_obstacle;
    considerLaneChange();
  } else if (lane_conflict && actual_distance <= critical_distance) {
    ref_vel -= max_acc;
    considerLaneChange();
    //std::cout << "following vehilce " << conflict_idx <<" with speed "<< vel_obstacle <<"\n";
  }
  return ref_vel;
}

// speed in m/s
double PathPlanner::convertSpeed2Increment(double speed, double target_x, double target_y)
{
  double dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double N = dist / (speed * 0.02);

  return target_x / N;
}


void PathPlanner::considerLaneChange()
{
  vector<int> lanesToCheck;
  if (curr_lane_idx == 0 || curr_lane_idx == 2) {
    lanesToCheck.push_back(1);
  } else if (curr_lane_idx == 1) {
    lanesToCheck.push_back(0);
    lanesToCheck.push_back(2);
  }

  for (int lane : lanesToCheck) {
    bool lane_good = true;
    for (vector<double> vehicle : sensor_fusion) {
      int obstacle_lane = check_lane_idx(vehicle[6]);
      double& obstacle_s = vehicle[5];
      double obstacle_vel = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
      double dist = obstacle_s - car_s;

      // check if car in that lane is in front of us and slower
      if (obstacle_lane == lane && obstacle_vel < mph2mps(car_speed) && dist > -10) {
        lane_good = false;
        break;
        // check if car is in that lane and faster but with not enough space to pull out
      } else if (obstacle_lane == lane && obstacle_vel > mph2mps(car_speed) &&
          (dist > -10 & dist < 10)) {
        lane_good = false;
        break;
      }
    }
    if (lane_good) {
      target_lane_idx = lane;
      std::cout << "Changing lane to " << lane << std::endl;
    }
  }
  return;
}




