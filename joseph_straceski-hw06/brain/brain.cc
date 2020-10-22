
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <deque>
#include <cmath>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;


std::vector<std::vector<bool>> hit_lookup;
std::vector<float> coll = {0, 0, 0};
#define PI 3.14159265


void print_arr(std::vector<float> v) {
  for (auto elem : v) {
    cout << elem << " ";
  }
  cout << endl;
}

float var = 0.05f;

bool aprox(float current_val, float last_val, float* guess_val, float* adjust, float push, bool loop, bool offset, float val) {
  float delt_val = current_val - last_val;

  if (loop && *guess_val > PI) {
    *guess_val -= PI;
  }

  if (loop && *guess_val < -PI) {
    *guess_val += PI;
  }

  if (delt_val == 0) {
    return false; 
  }

  float thresh_hold = std::min(abs(*guess_val * var + push) / 2.0f, abs(current_val * var + push) / 2.0f);

  if (abs(delt_val) < thresh_hold && !offset) {
    return false;
  }

  float out_guess_high = last_val + val + (*guess_val * var) + push;
  float out_mid = last_val + val;
  float out_guess_low  = last_val + val - (*guess_val * var) - push;

  float out_guess = out_guess_high;
  float scalar = 1;

  if (offset && abs(current_val - out_mid) < abs(current_val - out_guess_low) && abs(current_val - out_mid) < abs(current_val - out_guess_high)) {
    *adjust = abs(current_val - out_mid) * 5;
    return true;
  } else if (offset && abs(current_val - out_guess) > abs(current_val - out_guess_low)) {
    out_guess = out_guess_low;
    scalar = -1;
  } else if ((delt_val < 0) ^ (push + current_val * 0.05 < 0)) {
    out_guess = out_guess_low;
    scalar = -1;
  } 

  *adjust = clamp(0.01 * push, abs(current_val - out_guess) * 20, 50 * push);

  if (current_val > out_guess) {
    *guess_val += *adjust * scalar;
  } else if (current_val < out_guess) {
    *guess_val += -*adjust * scalar;
  }

  return true;
}

std::vector<float> cheat_input = {0, 0, 0};
std::vector<float> cheat_err = {0, 0, 0};
std::vector<float> cheat_err_d = {0, 0, 0};
std::vector<float> guess = {0, 0, 0};
std::vector<float> delt = {0, 0, 0};
std::vector<float> last = {0, 0, 0};
std::vector<float> avg_var = {0.2, 0.2, 0.05};
std::vector<float> min_accy = {0.05, 0.05, 0.01};
std::vector<float> loop = {false, false, true};
std::vector<float> dirty_input = {0, 0, 0};
std::vector<bool> trig = {false, false, false};

bool init_val = false;
bool coll_delete = false;
float time_delt = 0;
float time_delta = 0;


bool validate_coords(std::vector<float> dirty) {
  bool result = true;
  for (int idx = 0; idx < 3; ++idx) {
    if (init_val) {
      bool out = false;

      if (coll[idx] > min_accy[idx]) {
        trig[idx] = false;
      }

      if (abs(time_delta) > 1000 && abs(coll[idx]) > 0) {
        std::vector<float> p_val(21);
        std::vector<float> delts(21);
        std::vector<float> p_outs(21);
        float min_delt = FLT_MAX;
        float p_out = 0;
        int g = 0;
        

        for (int i = -10; i <= 10; ++i) {

          float offset = (coll[idx]) * 1.0f + 0.05 * float(i);
          float p_out_lst = guess[idx] + coll[idx];
          bool out_val = aprox(dirty[idx], last[idx], &p_out_lst, &delt[idx], avg_var[idx], loop[idx], true, offset);
          delts[i + 10] = delt[idx];
          p_val[i + 10] = guess[idx];
          p_outs[i + 10] = p_out_lst;
          if (delt[idx] <= min_delt) {
            p_out = p_out_lst;
            min_delt = float(delt[idx]);
            out = out_val;
            g = i + 10;
          }
        }

        guess[idx] = p_out;
        delt[idx] = min_delt;
        coll_delete = true;
        
      } else {
        out = aprox(dirty[idx], last[idx], &guess[idx], &delt[idx], avg_var[idx], loop[idx], false, 0);
      }

      trig[idx] = trig[idx] | out;
    }

    init_val = true;
    last[idx] = dirty[idx];
    result = result && trig[idx] && delt[idx] < min_accy[idx];
  }

  return result;
}

float grid_div = 0.25f;
int grid_offset = 28;
int size = std::floor((float(grid_offset) * 2.0)/grid_div);
float radius = float(grid_offset);

void draw_point(float x, float y, float r, float g, float b) {

  float x_delt = radius - x;
  float y_delt = radius - y;

  float ang = atan2(y_delt, x_delt);
  float dist = sqrt((y_delt * y_delt) + (x_delt * x_delt));

  viz_hit(-dist/float(radius) * 1.9f, ang, r, g, b);
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

  if (min > 0.5f) {
    std::vector<float> out = {x, y};
    point_list->emplace_back(out);
    draw_point(x, y, r, g, b);
  }
}


std::vector<std::vector<float>> master_wall_points_dirty;
std::vector<std::vector<float>> master_wall_points_clean;
std::vector<std::vector<float>> master_car_points;
std::vector<float> err_input = {0, 0, 0};
Matrix<float> pred_val;

bool stop = false;
bool setup = true;

float chassis_dy   = 0.216f;
float wheel_radius = 0.050f;
float wheel_width  = 0.040f;
float wheel_y0     = (chassis_dy*0.5f + wheel_width*0.6f + 0.010f);
float wheel_tread = wheel_y0 * 2;

float l_slp = 0.1f;
float r_slp = 0.1f;
float cg_y = 0;

bool last_cheat_set = false;
float last_time = 0;






void cmd_vel(Robot* robot, float vx, float vy, bool print) {
  Matrix< float > rot_mat( 2, 1 );
  Matrix< float > skid_mat( 3, 2 );
  Matrix< float > dist_mat ( 3, 1 );

  float const rot_data[ 2 ] = {vx, vy};
  float const skid_data[ 2 * 3 ] =
  {
    wheel_radius / 2.0f * l_slp, wheel_radius / 2.0f * r_slp,
    wheel_radius * cg_y / wheel_tread * l_slp, -wheel_radius * cg_y / wheel_tread * r_slp,
    wheel_radius / wheel_tread * l_slp, -wheel_radius / wheel_tread * r_slp
  };

  rot_mat = rot_data;
  skid_mat = skid_data;
  dist_mat = skid_mat * rot_mat * (1.0f/time_delta);

  float turn_dir = 1;

  if (vx > vy){
    turn_dir = -1;
  }

  if (coll_delete) {
    coll_delete = false;
    coll[0] = 0;
    coll[1] = 0;
    coll[2] = 0;
  }

  if (abs(time_delta) < 1000) {
    coll[0] += vx * (time_delta) * wheel_radius * PI * 2 * 0.61f * 1.04f * cos(guess[2]);
    coll[1] += vy * (time_delta) * wheel_radius * PI * 2 * 0.61f * 1.04f * sin(guess[2]);
  }

  if (abs(time_delta) < 1000) {
    coll[2] += -((vx - vy) * time_delta / 2.0f * wheel_radius * PI * 2.0f * 4.0f * 0.166f * 1.04f / (wheel_y0 * PI * 2));
  }

  robot->set_vel(vx, vy);
}

int pre_test = 4;
int wait_test = 20;
int test_length = 30;
int test_count = 0;
bool not_validated = true;
int col_frq = 2;
bool passed_validation = false;
std::vector<std::vector<int>> tile_grid(size, std::vector<int>(size, -1));

bool is_adj(int x, int y) {
  int ax = x + 1;
  int ay = y + 0;

  int bx = x - 1;
  int by = y + 0;

  int cx = x + 0;
  int cy = y + 1;

  int dx = x + 0;
  int dy = y - 1;

  int ex = x + 1;
  int ey = y + 1;

  int fx = x - 1;
  int fy = y + 1;

  int gx = x + 1;
  int gy = y - 1;

  int hx = x - 1;
  int hy = y - 1;

  if (ax < 0 || ay < 0 || ax >= size || ay >= size) {
    if (tile_grid[ax][ay] > 0) {
      return true;
    }
  }

  if (bx < 0 || by < 0 || bx >= size || by >= size) {
    if (tile_grid[bx][by] > 0) {
      return true;
    }
  }

  if (cx < 0 || cy < 0 || cx >= size || cy >= size) {
    if (tile_grid[cx][cy] > 0) {
      return true;
    }
  }

  if (dx < 0 || dy < 0 || dx >= size || dy >= size) {
    if (tile_grid[dx][dy] > 0) {
      return true;
    } 
  }


  if (ex < 0 || ey < 0 || ex >= size || ey >= size) {
    if (tile_grid[ex][ey] > 0) {
      return true;
    }
  }

  if (fx < 0 || fy < 0 || fx >= size || fy >= size) {
    if (tile_grid[fx][fy] > 0) {
      return true;
    }
  }

  if (gx < 0 || gy < 0 || gx >= size || gy >= size) {
    if (tile_grid[gx][gy] > 0) {
      return true;
    }
  }

  if (hx < 0 || hy < 0 || hx >= size || hy >= size) {
    if (tile_grid[hx][hy] > 0) {
      return true;
    } 
  }

  return false;
}


bool add_path (std::vector<std::deque<std::vector<int>>>* paths, std::deque<std::vector<int>>* path, 
  bool* used, std::deque<std::vector<int>>* found_path,
  std::vector<std::vector<bool>>* not_checked,
  int x, int y) {

  if (x < 0 || y < 0 || x >= size || y >= size) {
    return false;
  }

  int ax = x + 1;
  int ay = y + 0;

  int bx = x - 1;
  int by = y + 0;

  int cx = x + 0;
  int cy = y + 1;

  int dx = x + 0;
  int dy = y - 1;

  if (is_adj(x, y)) {
    return false;
  }

  if ((*not_checked)[x][y]) {
      if (*used) {
        std::deque<std::vector<int>> new_path(*path);
        (*not_checked)[x][y] = false;
        std::vector<int> point = {x, y};
        new_path.emplace_front(point);
        paths->emplace_back(new_path);
        if (tile_grid[x][y] < 0) {
          *found_path = new_path;
          return true;
        }
      } else {
        (*not_checked)[x][y] = false;
        std::vector<int> point = {x, y};
        path->emplace_front(point);
        *used = true;
        if (tile_grid[x][y] < 0) {
          *found_path = *path;
          return true;
        }
      }
    }

    return false;
}

std::deque<std::vector<int>> get_target(float xf, float yf) {
  std::vector<std::vector<bool>> not_checked(size, std::vector<bool>(size, true));
  std::vector<std::deque<std::vector<int>>> paths;

  int x1 = std::floor((grid_offset + xf) / grid_div);
  int y1 = std::floor((grid_offset + yf) / grid_div);
  bool found_new_tile = true;
  std::deque<std::vector<int>> path_try = {{x1, y1}};

  paths.emplace_back(path_try);
  int path_idx = 0;
  std::deque<std::vector<int>> found_path;
  while (found_new_tile) {
    std::deque<std::vector<int>> path = paths[path_idx];
    bool used = false;

    int xo = path[0][0];
    int yo = path[0][1];

    int ax = xo + 1;
    int ay = yo + 0;

    int bx = xo - 1;
    int by = yo + 0;

    int cx = xo + 0;
    int cy = yo + 1;

    int dx = xo + 0;
    int dy = yo - 1;


    if (add_path(&paths, &path, &used, &found_path, &not_checked, ax, ay)) {
      break;
    } else if (add_path(&paths, &path, &used, &found_path, &not_checked, bx, by)) {
      break;
    } else if (add_path(&paths, &path, &used, &found_path, &not_checked, cx, cy)) {
      break;
    } else if (add_path(&paths, &path, &used, &found_path, &not_checked, dx, dy)) {
      break;
    }

    if (!used) {
      path_idx += 1;
    }

  }

  return found_path;
}

void bresenham(float xf1, float yf1, float xf2, float yf2, bool hit) { 
  int x1 = std::floor((grid_offset + xf1) / grid_div);
  int y1 = std::floor((grid_offset + yf1) / grid_div);
  int x2 = std::floor((grid_offset + xf2) / grid_div);
  int y2 = std::floor((grid_offset + yf2) / grid_div);

  float m_new = 2 * (y2 - y1); 
  float slope_error_new = m_new - (x2 - x1); 

  int last_x = x1;
  int last_y = y1;
  bool init = false;

  for (int x = x1, y = y1; x <= x2; x += 1) { 
    // cout << x1 << " " << y1 << " " << x2 << " " << y2 << " " <<  endl;
    // cout << xf1 << " " << yf1 << " " << xf2 << " " << yf2 << " " <<  endl;
    
    if (init && tile_grid[last_x][last_y] > 0) {
      break;
    }

    if (init && tile_grid[last_x][last_y] < 0) {
      tile_grid[last_x][last_y] = 0;
      draw_point((last_x * grid_div), (last_y * grid_div), 0, 0, 0);
    }

    init = true;

    last_x = x;
    last_y = y;

    // Add slope to increment angle formed 
    slope_error_new += m_new; 

    // Slope error reached limit, time to 
    // increment y and update slope error. 
    if (slope_error_new >= 0) { 
       y++; 
       slope_error_new  -= 2 * (x2 - x1); 
    } 
  }

  
  if (hit) {
    tile_grid[last_x][last_y] = 1;
    draw_point((last_x * grid_div), (last_y * grid_div), 1, 0, 0);
  } else if (tile_grid[last_x][last_y] < 0) {
    tile_grid[last_x][last_y] = 0;
    draw_point((last_x * grid_div), (last_y * grid_div), 0, 0, 0);
  }
} 

void setup_stage (Robot* robot) {
  if (!passed_validation) {
    cout << " validating " << test_count << endl; 
    cmd_vel(robot, 0.0, 0.0, true);
    // print_arr(guess);
    // print_arr(cheat_input);
    passed_validation = validate_coords(dirty_input);
    return;
  }
  setup = false;
}




void draw_debug(Robot* robot) {
  draw_point(cheat_input[0], cheat_input[1], 0.0, 0.0, 0.0);
  for(const auto hit: robot->ranges) {
    if (hit.range > 10) {
      continue;
    }

    float x_val2 = cheat_input[0] + hit.range * cos(hit.angle + cheat_input[2]);
    float y_val2 = cheat_input[1] + hit.range * sin(hit.angle + cheat_input[2]);
    add_point(x_val2, y_val2, 1.0, 0.0, 0.0, &master_wall_points_clean);
  }
}

int check_count = 0;
int check_limit = 25;
int check_count2 = 0;
int check_limit2 = 200;

std::deque<std::vector<int>> target_path;
std::vector<int> target_point = {};
bool has_path = false;

bool drive_to_point (Robot* robot, std::vector<int> point) {

  float dx = (float(point[0] * grid_div) - grid_offset) - guess[0];
  float dy = (float(point[1] * grid_div) - grid_offset) - guess[1];
  float ang = atan2(dy, dx) * 2;
  float tang = ang - guess[2];
  if (tang > PI) {
    tang -= PI * 2;
  } else if (tang < -PI) {
    tang += PI * 2;
  }

  if (tang < -0.1){
    cmd_vel(robot, 1.0, -1.0, false);
    cout << "rotating " << ang << " " << guess[2] << endl;
    return false;
  } else if (tang > 0.1){
    cmd_vel(robot, -1.0, 1.0, false);
    cout << "rotating " << ang << " " << guess[2] << endl;
    return false;
  } else if (dx * dx + dy * dy > 0.25 * 0.25) {
    cout << "driving" << ang << " " << guess[2] << endl;
    if (robot->ranges[3].range < 0.5) {
      return true;
    } 
    cmd_vel(robot, 1.0, 1.0, false);
    return false;
  }
  return true;
}

void main_drive(Robot* robot) {
  bool result = validate_coords(dirty_input);
  if (check_count > check_limit && !result) {
    cmd_vel(robot, 0.0, 0.0, false);
    check_count = 0;
  } else {

    if (result) {
      for(const auto hit: robot->ranges) {
        float r = hit.range;
        if (std::isinf(r)) {
          r = 2.0f;
        }

        float x_val1 = guess[0] + clamp(0.0f, r, 2.0f) * cos(hit.angle + guess[2]);
        float y_val1 = guess[1] + clamp(0.0f, r, 2.0f) * sin(hit.angle + guess[2]);
        bresenham(guess[0], guess[1], x_val1, y_val1, r < 2.0f);
      }
    }

    check_count++;

    if (check_count2 > check_limit2) {
      if (!has_path) {
        cout << " o " << endl;
        target_path = get_target(guess[0], guess[1]);

        if (target_path.size() > 0) {
          has_path = true;
          target_point = target_path.back();
          target_path.pop_back();
        }
      } else if (target_path.size() > 0) {
        float dx = float(target_point[0] * grid_div) - float(grid_offset);
        float dy = float(target_point[1] * grid_div) - float(grid_offset);
        cout << "driving to = [" << dx << ", " << dy << "] p_left: [" << target_path.size() << "] current = [" << guess[0] << " " <<  guess[1] << "]" << endl;
        if (tile_grid[target_point[0]][target_point[1]] > 0 || is_adj(target_point[0],target_point[1])) {
          check_count2 = 0;
          has_path = false;
        } else if (drive_to_point(robot, target_point)) {
          if (target_path.size() > 0) {
            target_point = target_path.back();
            target_path.pop_back();
          } else {
            check_count2 = 0;
            has_path = false;
          }
        }
      } else {
        check_count2 = 0;
        has_path = false;
      }
      cout << "?" << endl;
      return;
    }

    cout << "cc:" << check_count2 << " c2:" << check_count << endl;

    
    check_count2++;

    float rgt = clamp(0.0, robot->ranges[1].range, 2.0);
    float frgt = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float flft = clamp(0.0, robot->ranges[4].range, 2.0);
    float lft = clamp(0.0, robot->ranges[5].range, 2.0);
    float blft = clamp(0.0, robot->ranges[6].range, 2.0);
    float brgt = clamp(0.0, robot->ranges[8].range, 2.0);
    float brgt2 = clamp(0.0, robot->ranges[9].range, 2.0);
    float brgt3 = clamp(0.0, robot->ranges[10].range, 2.0);

    // std::vector<float> out = {lft, flft, fwd, frgt, rgt, blft, brgt, brgt2, brgt3};
    // print_arr(out);
    

    if (fwd < 1.0) {
      if (lft < 1.0 && flft > 1.8) {
        cout << "A1" << endl;
        cmd_vel(robot, -1, 1, false);
      } else if (rgt < 1.0 && frgt > 1.8) {
        cout << "A2" << endl;
        cmd_vel(robot, 1, -1, false);
      } else if (rgt + frgt > lft + flft) {
        cout << "B1" << endl;
        cmd_vel(robot, 1, -1, false);
      } else {
        cout << "B2" << endl;
        cmd_vel(robot, -1, 1, false);
      }
    } else if (rgt < 1.8 && rgt > 1.1 && frgt > rgt && blft > 1.4) {
      cout << "C" << endl;
      cmd_vel(robot, 1, -1, false);
    } else if ((frgt / sqrt(2) < 0.7 || rgt < 0.7)) {
      if (frgt / sqrt(2) < 0.5 && rgt < frgt / sqrt(2)) {
        cmd_vel(robot, 1, 1, false);
      } else if (rgt / 1.3 > frgt / sqrt(2) || frgt / sqrt(2) < 0.5) {
        cmd_vel(robot, -1, 1, false);
        cout << "D1" << endl;
      } else if (rgt * 1.3 < frgt / sqrt(2) && frgt < 0.8) {
        cmd_vel(robot, 1, -1, false);
        cout << "D2" << endl;
      } else {
        cmd_vel(robot, 1, 1, false);
      }
    } else if ( lft < 1.8 && lft > 1.1 && flft > lft) {
      cout << "E" << endl;
      cmd_vel(robot, -1, 1, false);
    } else if ((flft / sqrt(2) < 0.7 || lft < 0.7)) {
      if (flft / sqrt(2) < 0.5 && lft < flft / sqrt(2)) {
        cmd_vel(robot, 1, 1, false);
      } else if (lft / 1.3 > flft / sqrt(2) || flft / sqrt(2) < 0.5) {
        cmd_vel(robot, 1, -1, false);
        cout << "F1" << endl;
      } else if (lft * 1.3 < flft / sqrt(2) && flft < 0.8) {
        cmd_vel(robot, -1, 1, false);
        cout << "F2" << endl;
      } else {
        cmd_vel(robot, 1, 1, false);
      }
    } else if (blft < 1.5 && lft > 1.5 && rgt > 1.2) {
      cmd_vel(robot, -1, 1, false);
      cout << "G" << endl;
    } else {
      cmd_vel(robot, 1, 1, false);
      cout << "fwd" << endl;
    }
  }
}

void callback(Robot* robot) {
  dirty_input = {robot->pos_x, robot->pos_y, robot->pos_t};
  // cheat_input = {robot->raw_x, robot->raw_y, robot->raw_t};
  // cheat_err_d = {robot->err_x - cheat_err[0], robot->err_y - cheat_err[1], robot->err_t - cheat_err[2]}; 
  // cheat_err = {robot->err_x, robot->err_y, robot->err_t}; 
  time_delta = robot->stamp - last_time;
  last_time = robot->stamp;
  print_arr(cheat_input);
  print_arr(cheat_input);
  if (setup) {
    setup_stage(robot);
  } else {
    main_drive(robot);
  } 
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
