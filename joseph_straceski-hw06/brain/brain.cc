
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <cmath>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;


std::vector<std::vector<bool>> hit_lookup;

// struct node {
//   float x, y;


// }




float get_error(float x, float y, float turn, float err_thresh, auto points, std::vector<std::vector<float>>* arr) {
  float err = 0;

  for(const auto hit: points) {
    float min = 0;

    if (arr->size() > 0){
      min = FLT_MAX;
    }
    
    float x_corr = x + hit.range * sin(hit.angle + turn);
    float y_corr = y + hit.range * cos(hit.angle + turn);

    for(const std::vector<float>& master: *arr) {
      float x_dlt = (x_corr - master[0]);
      float y_dlt = (y_corr - master[1]);

      float dist = x_dlt * x_dlt + y_dlt * y_dlt;
      
      if (dist < min) {
        min = dist;
      }
    }

    err += min;
  }

  return err;
}


std::vector<float> correction_box(float x_dirty, float y_dirty, float t_dirty, auto points, std::vector<std::vector<float>>* arr) {
  std::vector<float> max_pos = {(x_dirty + 2.0) / 0.50, (y_dirty + 2.0) / 0.50};
  std::vector<float> min_pos = {(x_dirty - 2.0) / 1.50, (y_dirty - 2.0) / 1.50};

  float max_turn = (t_dirty + 0.5) / 0.50;
  float min_turn = (t_dirty - 0.5) / 1.50;

  std::vector<float> pos_window = {max_pos[0] - min_pos[0], max_pos[1] - min_pos[1]};
  float turn_window = max_turn - min_turn;

  float div = 10;

  std::vector<float> delt_pos = {pos_window[0]/div, pos_window[1]/div};
  float delt_turn = turn_window / div;

  float x_best = -1;
  float y_best = -1;
  float t_best = -1;
  float min_err = -1;
  float error_thresh = (delt_pos[0] * delt_pos[0]) / 2;

  for (int t = 0; t < div; ++t) {
    for (int x = 0; x < div; ++x) {
      for (int y = 0; y < div; ++y) {

        float x_val = min_pos[0] + (delt_pos[0] * x);
        float y_val = min_pos[1] + (delt_pos[1] * y);
        float t_val = min_turn + (delt_turn * t);

        float err = get_error(x_val, y_val, t_val, delt_pos[0], points, arr);

        if (min_err < 0 || err < min_err) {
          x_best = x_val;
          y_best = y_val;
          t_best = t_val;

          min_err = err;
        }

        if (min_err < error_thresh) {
          break;
        }
      }
      if (min_err < error_thresh) {
        break;
      }
    }
    if (min_err < error_thresh) {
      break;
    }
  }

  return {x_best, y_best, t_best};
}

float min_dist = 1.0;
float x_size = 70.0;
float y_size = 70.0;

float x_mid = x_size / 2.0;
float y_mid = y_size / 2.0;

void draw_point(float x, float y, float r, float g, float b) {
  
  float dec = 2;
  float exp = std::pow(10.0, dec);
  
  x = std::round(x * exp) / exp;
  y = std::round(y * exp) / exp;

  float x_delt = x_mid - x;
  float y_delt = y_mid - y;

  float ang = atan2(y_delt, x_delt);
  float dist = sqrt((y_delt * y_delt) + (x_delt * x_delt));

  viz_hit(-dist/y_mid, ang, r, g, b);

  cout << x << " " << y << " " << r << " " << g << " " << b << endl;
}

void add_point(float x, float y, float r, float g, float b, std::vector<std::vector<float>>* point_list) {
  float min = FLT_MAX;
  
  for(const std::vector<float>& master: *point_list) {
    float x_dlt = (x - master[0]);
    float y_dlt = (y - master[1]);

    float dist = x_dlt * x_dlt + y_dlt * y_dlt;
    
    if (dist < min) {
      min = dist;
    }
  }

  if (min > min_dist) {
    std::vector<float> out = {x, y};
    point_list->emplace_back(out);
    draw_point(x, y, r, g, b);
  }
}

std::vector<std::vector<float>> master_wall_points_dirty;
std::vector<std::vector<float>> master_wall_points_clean;
std::vector<std::vector<float>> master_car_points;
bool stop = false;

void callback(Robot* robot) {
    // cout << "\n===" << endl;

    std::vector<float> param1 = correction_box(robot->pos_x, robot->pos_y, robot->pos_t, robot->ranges, &master_wall_points_dirty);
    std::vector<float> param2 = {robot->raw_x, robot->raw_y, robot->raw_t};

    if (param1[0] == -1 || param1[1] == -1 || param1[2] == -1) {
      cout << param1[0] << " "  << param1[1] << " " << param1[2] << endl;
    }

    
    add_point(param2[0], param2[1], 0.0, 0.0, 0.0, &master_car_points);

    for(const auto hit: robot->ranges) {
      
      
      

      // cout << x_val << " " << y_val  << " " << hit.angle << " " << hit.range << endl;
      // cout << (30.0  + x_val) / 60.0 << " " << (30.0 + y_val) / 60.0  << endl;

      if (hit.range > 10) {
        continue;
      }

      float x_val1 = param1[0] + hit.range * cos(hit.angle + param1[2]);
      float y_val1 = param1[1] + hit.range * sin(hit.angle + param1[2]);

      float x_val2 = param2[0] + hit.range * cos(hit.angle + param2[2]);
      float y_val2 = param2[1] + hit.range * sin(hit.angle + param2[2]);

      add_point(x_val1, y_val1, 0.0, 1.0, 0.0, &master_wall_points_dirty);
      add_point(x_val2, y_val2, 1.0, 0.0, 0.0, &master_wall_points_clean);
    }

    if (robot->ranges.size() < 5) {
        robot->set_vel(2.0, 2.0);
        return;
    }

    float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float rgt = clamp(0.0, robot->ranges[4].range, 2.0);
    // cout << "lft,fwd,rgt = "
    //      << lft << ","
    //      << fwd << ","
    //      << rgt << endl;

    float spd = fwd - 1.0;
    float trn = clamp(-1.0, lft - rgt, 1.0);

    if (fwd < 1.2) {
      spd = 0;
      trn = 1;
    }

    // cout << "spd,trn = " << spd << "," << trn << endl;
    robot->set_vel(spd + trn, spd - trn);



    // tree = node.get_neighbors(tree)


    /*
    cout << "x,y,t = "
         << robot->pos_x << ","
         << robot->pos_y << ","
         << robot->pos_t << endl;
    robot->set_vel(robot->pos_t, -robot->pos_t);
    */
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);

    return viz_run(argc, argv);
}
