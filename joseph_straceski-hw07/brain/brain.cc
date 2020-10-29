
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
std::vector<float> drive_shift = {0, 0, 0};
std::vector<float> save_guess = {0, 0, 0};

#define PI 3.14159265


void print_arr(std::vector<float> v) {
  for (auto elem : v) {
    cout << elem << " ";
  }
  cout << endl;
}

std::vector<float> guess = {0, 0, 0};
std::vector<float> offset = {0, 0, 0};

bool init_val = false;
bool coll_delete = false;
float time_delt = 0;
float time_delta = 0;


float grid_div = 0.19f;
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


void draw_path (std::deque<std::vector<int>> d_path) {
  for (int i = 0; i < d_path.size(); ++i)
  {
    std::vector<int> point = d_path[i];
    draw_point(float(point[0] * grid_div), float(point[1] * grid_div), 0.0f, 1.0f, 1.0f);
  }
}


std::vector<std::vector<float>> master_wall_points_dirty;
std::vector<std::vector<float>> master_wall_points_clean;
std::vector<std::vector<float>> master_car_points;
std::vector<float> err_input = {0, 0, 0};

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

std::vector<float> last_drive = {0, 0};
std::vector<float> guess_diff = {0, 0};
std::vector<float> last_heading = {0, 0};

std::vector<float> last_drive_input= {2, 2};

void cmd_vel(Robot* robot, float vx, float vy, bool print) {


  float tot = guess_diff[0] * guess_diff[0] + guess_diff[1] * guess_diff[1];
  float coll_tot = coll[0] * coll[0] + coll[1] * coll[1];

  float pred_vec = guess_diff[0] * coll[0] + coll[1] * guess_diff[1];
  float d_s = 0.19 * 2;

  if (coll_delete && ((abs(pred_vec) > 0 && coll_tot > 0) || abs(coll[2] - guess_diff[2]) > 0.001)){
    float a1 = atan2(coll[1], coll[0]);
    float a2 = atan2(guess_diff[1], guess_diff[0]);
    float tang = (a1 - a2);

    if (guess_diff[2] > PI) {
      guess_diff[2] -= PI * 2;
    } else if (guess_diff[2] < -PI) {
      guess_diff[2] += PI * 2;
    }

    if (tang > PI) {
      tang -= PI * 2;
    } else if (tang < -PI) {
      tang += PI * 2;
    }

    cout << "@@@@@@@@@@@@@@ " << tang << " " << drive_shift[0] << " " << drive_shift[1]<< endl;
    if ((tang > 0.1 && abs(last_drive[0] + last_drive[1]) / 2.0f > 0.1f) || coll[2] < guess_diff[2]) {
      drive_shift[1] += d_s;
      drive_shift[0] -= d_s;
    }

    if ((tang < -0.1 && abs(last_drive[0] + last_drive[1]) / 2.0f > 0.1f ) || coll[2] > guess_diff[2]) {
      drive_shift[1] -= d_s;
      drive_shift[0] += d_s;
    }

    cout << guess_diff[0] << " " << guess_diff[1] << endl;
    cout << coll[0] << " " << coll[1] << endl;
    
    last_drive = {0, 0};
    cout << "@@@@@@@@@@@@@@ " << tang << " " << drive_shift[0] << " " << drive_shift[1]<< endl;

    if (tot < coll_tot - 0.001 || pred_vec < 0) {
        drive_shift[0] -= d_s;
        drive_shift[1] -= d_s;
    } else if (tot > coll_tot + 0.001) {
        drive_shift[0] += d_s;
        drive_shift[1] += d_s;
    }
    cout << "@@@@@@@@@@@@@@ " << tang << " " << drive_shift[0] << " " << drive_shift[1] << endl;
  }

  last_drive = {last_drive[0] + vx, last_drive[1] + vy};

  cout << "vx:" << vx << " vy:" << vy << endl;
  cout << "tot:" << tot << " ct:" << coll_tot << " t:" << coll_delete << " d" << abs(time_delta) << " l:" << pred_vec  << " " << guess_diff[2] << " as:" << coll[2] << endl;
  cout << "td:" << (drive_shift[0] * 1.05f / 0.2f) << " " << (drive_shift[1] * 1.05f / 0.2f) << " " << robot->err_l << " " << robot->err_r << endl;
  

  if (coll_delete) {
    coll_delete = false;
    coll[0] = 0;
    coll[1] = 0;
    coll[2] = 0; 
  }

  if (abs(time_delta) < 1000) {
    coll[0] +=  ((vx + vy) / 2.0f * time_delta * wheel_radius * PI * 2 * 0.68 * cos(guess[2]));
    coll[1] +=  ((vx + vy) / 2.0f * time_delta * wheel_radius * PI * 2 * 0.68 * sin(guess[2]));
    coll[2] += -((vx - vy) / 2.0f * time_delta * wheel_radius * PI * 2 * 0.68 / (wheel_y0 * PI * 2));
  }

  drive_shift[0] = clamp((-10.0f * 0.2f) / 1.05f,  drive_shift[0], (10.0f * 0.2f) / 1.05f);
  drive_shift[1] = clamp((-10.0f * 0.2f) / 1.05f,  drive_shift[1], (10.0f * 0.2f) / 1.05f);

  robot->set_vel(vx / 1.05f - drive_shift[0], vy / 1.05f - drive_shift[1]);
}

int pre_test = 4;
int wait_test = 20;
int test_length = 30;
int test_count = 0;
bool not_validated = true;
int col_frq = 2;
bool passed_validation = false;
std::vector<std::vector<float>> tile_grid(size, std::vector<float>(size, -1));

bool apply_adj(int x, int y, float v, std::vector<std::vector<float>> &grid) {
  int ax = x + 1;
  int ay = y + 0;

  int bx = x - 1;
  int by = y + 0;

  int cx = x + 0;
  int cy = y + 1;

  int dx = x + 0;
  int dy = y - 1;

  if (ax >= 0 && ay >= 0 && ax < grid.size() && ay < grid[0].size()) {
    if (grid[ax][ay] == -1) {
      grid[ax][ay] = 0;
    }
    grid[ax][ay] += v;
    grid[ax][ay] = clamp(-0.99, grid[ax][ay], 2.0);
  }

  if (bx >= 0 && by >= 0 && bx < grid.size() && by < grid[0].size()) {
    if (grid[bx][by] == -1) {
      grid[bx][by] = 0;
    }
    grid[bx][by] += v;
    grid[bx][by] = clamp(-0.99, grid[bx][by], 2.0);
  }

  if (cx >= 0 && cy >= 0 && cx < grid.size() && cy < grid[0].size()) {
    if (grid[cx][cy] == -1) {
      grid[cx][cy] = 0;
    }
    grid[cx][cy] += v;
    grid[cx][cy] = clamp(-0.99, grid[cx][cy], 2.0);
  }

  if (dx >= 0 && dy >= 0 && dx < grid.size() && dy < grid[0].size()) {
    if (grid[dx][dy] == -1) {
      grid[dx][dy] = 0;
    }
    grid[dx][dy] += v;
    grid[dx][dy] = clamp(-0.99, grid[dx][dy], 2.0);
  }
}

bool is_adj(int x, int y, int v, std::vector<std::vector<float>> &grid) {
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

  int width = grid.size();
  int height = grid[0].size();

  if (ax >= 0 && ay >= 0 && ax < width && ay < height) {
    if (tile_grid[ax][ay] > v) {
      return true;
    }
  }

  if (bx >= 0 && by >= 0 && bx < width && by < height) {
    if (tile_grid[bx][by] > v) {
      return true;
    }
  }

  if (cx >= 0 && cy >= 0 && cx < width && cy < height) {
    if (tile_grid[cx][cy] > v) {
      return true;
    }
  }

  if (dx >= 0 && dy >= 0 && dx < width && dy < height) {
    if (tile_grid[dx][dy] > v) {
      return true;
    } 
  }


  if (ex >= 0 && ey >= 0 && ex < width && ey < height) {
    if (tile_grid[ex][ey] > v) {
      return true;
    }
  }

  if (fx >= 0 && fy >= 0 && fx < width && fy < height) {
    if (tile_grid[fx][fy] > v) {
      return true;
    }
  }

  if (gx >= 0 && gy >= 0 && gx < width && gy < height) {
    if (tile_grid[gx][gy] > v) {
      return true;
    }
  }

  if (hx >= 0 && hy >= 0 && hx < width && hy < height) {
    if (tile_grid[hx][hy] > v) {
      return true;
    } 
  }
  return false;
}

bool plot_point_b(int x, int y, bool add, std::vector<std::vector<float>> &grid, float draw) {

  if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
    cout << x << " " << y << endl;
    return true;
  }

  if (grid[x][y] == -1) {
    grid[x][y] = 0;
  }

  if (grid[x][y] < -0.9) {
    grid[x][y] = -0.9;
  }

  if (add) {
    grid[x][y] += 0.45f;
    apply_adj(x, y, 0.1, grid);
  } else {
    grid[x][y] -= 0.45f;
    apply_adj(x, y, -0.1, grid);
    //apply_adj(x, y, -0.15, grid);
  }

  grid[x][y] = clamp(-0.99, grid[x][y], 2.0);

  if (draw) {
    draw_point((x * grid_div), (y * grid_div), clamp(0, grid[x][y], 1.0f) / 1.0f, 0, 0);
  }
  return false;
}

bool check_val(int x, int y, float val, bool adj, std::vector<std::vector<float>> &grid) {
  if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
    return true;
  }

  if (grid[x][y] > val || (adj && is_adj(x, y, val, grid))) {
    return true;
  }

  return false;
}


bool bresenham_int(int x1, int y1, int x2, int y2, bool hit, bool find, std::vector<std::vector<float>> &grid, bool draw) { 
  int delta_x(x2 - x1);
  // if x1 == x2, then it does not matter what we set here
  signed char const ix((delta_x > 0) - (delta_x < 0));
  delta_x = std::abs(delta_x) << 1;

  int delta_y(y2 - y1);
  // if y1 == y2, then it does not matter what we set here
  signed char const iy((delta_y > 0) - (delta_y < 0));
  delta_y = std::abs(delta_y) << 1;
  
  if (!find && plot_point_b(x1, y1, false, grid, draw)) {
    return false;
  }

  if (find && check_val(x1, y1, 0, false, grid)) {
    cout << "o1:" << x1 << " " << y1  << " " << tile_grid[x1][y1] <<  endl;
    return false;
  }

  bool init = false;
  int last_x = 0;
  int last_y = 0;

  if (delta_x >= delta_y)
  {
      // error may go below zero
      int error(delta_y - (delta_x >> 1));

      while (x1 != x2)
      {
          if (find && init && check_val(last_x, last_y, 0, true, grid)) {
            cout << "l1:" << last_x << " " << last_y <<  endl;
            return false;
          }

          if (!find && init && plot_point_b(last_x, last_y, false, grid, draw)) {
            return false;
          }

          // reduce error, while taking into account the corner case of error == 0
          if ((error > 0) || (!error && (ix > 0)))
          {
              error -= delta_x;
              y1 += iy;
          }
          // else do nothing

          error += delta_y;
          x1 += ix;

          last_x = x1;
          last_y = y1;
          
          init = true;
      }
  }
  else
  {
      // error may go below zero
      int error(delta_x - (delta_y >> 1));

      while (y1 != y2)
      {
          if (find && init && check_val(last_x, last_y, 0, true,  grid)) {
            cout << "l2:" << last_x << " " << last_y <<  endl;
            return false;
          }

          if (!find && init && plot_point_b(last_x, last_y, false, grid, draw)) {
            return false;
          }
          // reduce error, while taking into account the corner case of error == 0
          if ((error > 0) || (!error && (iy > 0)))
          {
              error -= delta_y;
              x1 += ix;
          }
          // else do nothing

          error += delta_x;
          y1 += iy;

          last_x = x1;
          last_y = y1;
          
          init = true;
      }
  } 
  
  if (last_x < 0 || last_y < 0 || last_x >= size || last_y >= size) {

    cout << "l3:" << last_x << " " << last_y <<  endl;
    return false;
  }



  if (!find) {
    if (hit) {
      plot_point_b(last_x, last_y, true, grid, draw);
    } else if (tile_grid[last_x][last_y] < 0) {
      plot_point_b(last_x, last_y, false, grid, draw);
    }
  }

  if (find && init && check_val(last_x, last_y, 0, false,  grid)) {
    cout << "l4:" << last_x << " " << last_y <<  endl;
    return false;
  }

  return true;
}

// code modified from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
bool bresenham(float xf1, float yf1, float xf2, float yf2, bool hit, bool find, std::vector<std::vector<float>> &grid, bool draw) { 
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  if (!find) {
    x1 = std::floor((grid_offset + xf1) / grid_div);
    y1 = std::floor((grid_offset + yf1) / grid_div);
    x2 = std::floor((grid_offset + xf2) / grid_div);
    y2 = std::floor((grid_offset + yf2) / grid_div);
  } else {
    x1 = xf1;
    y1 = yf1;
    x2 = xf2;
    y2 = yf2;
  }
  
  return bresenham_int(x1, y1, x2, y2, hit, find, grid, draw);
} 


std::deque<std::vector<int>> reduce_path(std::deque<std::vector<int>> input) {
  cout << " reduce " << input.size() << endl;

  for (int i = 0; i < input.size() - 1; ++i) {
    std::vector<int> start = input[i];

    if (i >= input.size() - 1) {
      break;
    }

    for (int j = i + 1; j < input.size() - 1; ++j) {
      std::vector<int> end = input[j];
      cout << start[0] << " " << start[1] << " " <<  end[0] <<  " " <<  end[1] << endl;
      if (bresenham(start[0], start[1], end[0], end[1], false, true, tile_grid, false)) {
        cout << "del:"  << j << endl;
        input.erase(input.begin() + j);
        j--;
      } else {
        i = j - 1;
        break;
      }
    }
  }

  for (int i = 0; i < input.size(); ++i)
  {
    std::vector<int> start = input[i];
    float dx = float(start[0] * grid_div) - float(grid_offset);
    float dy = float(start[1] * grid_div) - float(grid_offset);
    cout << dx <<  " " << dy << endl;
  }

  cout << " end " << input.size() << endl;
  return input;
}

bool add_path (std::vector<std::deque<std::vector<int>>>* paths, int path_idx, 
  bool* used, std::deque<std::vector<int>>* found_path,
  std::vector<std::vector<bool>>* not_checked,
  int x, int y, bool not_finished) {

  if (x < 0 || y < 0 || x >= size || y >= size) {
    return false;
  }

  if ((*not_checked)[x][y]) {
      (*not_checked)[x][y] = false;
      draw_point((x)/grid_div, float(y)/grid_div, 0, 0, 1.0f);


      if (is_adj(x, y, 0, tile_grid)) {
        return false;
      }

      std::vector<int> point = {x, y};

      if (*used) {
        std::deque<std::vector<int>> new_path((*paths)[path_idx]);
        new_path.emplace_front(point);
        paths->emplace_back(new_path);

        if (!is_adj(x, y, -1, tile_grid) && tile_grid[x][y] < 0) {
          *found_path = new_path;
          return true;
        }
      } else {
        
        if (not_finished) {
          std::deque<std::vector<int>> new_path((*paths)[path_idx]);
          paths->emplace_back(new_path);
        }

        (*paths)[path_idx].emplace_front(point);
        *used = true;

        if (!is_adj(x, y, -1, tile_grid) && tile_grid[x][y] < 0) {
          *found_path = (*paths)[path_idx];
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
  std::deque<std::vector<int>> path_try = {{x1, y1}};

  paths.emplace_back(path_try);
  int path_idx = 0;
  std::deque<std::vector<int>> found_path = {{x1, y1}};
  while (path_idx < paths.size()) {
    bool used = false;

    int xo = paths[path_idx][0][0];
    int yo = paths[path_idx][0][1];

    int ax = xo + 0;
    int ay = yo + 1;

    int bx = xo + 1;
    int by = yo + 0;

    int cx = xo + 0;
    int cy = yo - 1;

    int dx = xo - 1;
    int dy = yo + 0;


    if (add_path(&paths, path_idx, &used, &found_path, &not_checked, ax, ay, true)) {
      break;
    } else if (add_path(&paths, path_idx, &used, &found_path, &not_checked, bx, by, true)) {
      break;
    } else if (add_path(&paths, path_idx, &used, &found_path, &not_checked, cx, cy, true)) {
      break;
    } else if (add_path(&paths, path_idx, &used, &found_path, &not_checked, dx, dy, false)) {
      break;
    }

    if (!used) {
      path_idx += 1;
    }

  }

  draw_path(found_path);
  return found_path;
}

void setup_stage (Robot* robot) {
  if (!passed_validation) {
    cout << " validating " << test_count << endl; 
    cmd_vel(robot, 0.0, 0.0, true);
    // print_arr(guess);
    // print_arr(cheat_input);
    // passed_validation = validate_coords(dirty_input);
    return;
  }
  setup = false;
}

std::vector<float> last_pos = {0, 0};
std::vector<float> this_pos = {0, 0};

std::vector<float> last_guess = {0, 0};
std::vector<int> last_err = {0, 0};
int c = 0;


void fix_guess(Robot* robot) {
  cout << robot->err_x << " " << robot->err_y << endl;
  // if (abs(time_delta) < 1000) {
  //   return;
  // }
  cout << "A" << endl;


  int x1 = std::floor((grid_offset + guess[0]) / grid_div);
  int y1 = std::floor((grid_offset + guess[1]) / grid_div);
  int range = 4;//std::ceil(4.0 / grid_div);

  float w_range = 2.5;
  int window = std::ceil((w_range * 2 + grid_div) / grid_div);
  int window_shift = std::ceil(window / 2.0f);

  std::vector<std::vector<float>> relative_map(window, std::vector<float>(window, -1.0f));;


  for(const auto hit: robot->ranges) {
    float r = hit.range;
    if (std::isinf(r) || r > 2) {
      r = 2.0001f;
    }

    // cout << r << endl;
    float x2 = std::floor(r * cos(hit.angle + guess[2]) / grid_div);
    float y2 = std::floor(r * sin(hit.angle + guess[2]) / grid_div); 

    bresenham_int(window_shift, window_shift, x2 + window_shift, y2 + window_shift, r <= 2.0f, false, relative_map, false);
  }

  // cout << "B:" << robot->ranges.size() << endl;
 
  float shift_x = 0;
  float shift_y = 0;

  int best_x = 0;
  int best_y = 0;

  float min_err = FLT_MAX;
  float max_gain = -FLT_MAX;
  float min_err_none = 0.15;

  std::vector<float> c_col = coll;
  if (abs(time_delta) < 1000) {
    c_col = {0, 0, 0};
  } else {
    last_heading = {guess[2]};
    coll_delete = true;
  }

  

  for (int i1 = -range; i1 <= range; ++i1)
  {
    for (int i2 = -range; i2 <= range; ++i2)
    {
      float err = 0;
      float hit = 0;
      for (int j1 = 0; j1 < window; ++j1)
      {
        for (int j2 = 0; j2 < window; ++j2)
        {
          int x_actual = x1 - window_shift + i1 + j1;
          int y_actual = y1 - window_shift + i2 + j2;

          if (x_actual < 0 || x_actual >= size || y_actual < 0 || y_actual >= size) {
            continue;
          }

          if (relative_map[j1][j2] == -1 || tile_grid[x_actual][y_actual] == -1) {
            continue;
          } else if ((relative_map[j1][j2] > 0) ^ (tile_grid[x_actual][y_actual] > 0)) {
            err += abs(relative_map[j1][j2] * tile_grid[x_actual][y_actual]);
          } else {
            hit += abs(relative_map[j1][j2] * tile_grid[x_actual][y_actual]);
          }
        }
      }

      float cum_err = abs((guess[0] + i1 * grid_div) - (last_guess[0] + c_col[0])) * 20.0f
          + abs((guess[1] + i2 * grid_div) - (last_guess[1] + c_col[1])) * 20.0f + err - (hit/10.0f);

      // cout << "i1:" << i1 << "\ti2:" << i2 << "\tdx:" << abs((guess[0] + i1 * grid_div) - (last_guess[0] + coll[0])) * 5 << "\tdy:" 
      //     << abs((guess[1] + i2 * grid_div) - (last_guess[1] + coll[1])) * 5 << "\terr:" << err << "\thit:" << (hit/10.0f) << "\tcum:" << cum_err << endl;

      if (hit > 0) {
        if (cum_err < min_err) {
          best_x = i1;
          best_y = i2;
          min_err = cum_err;
          max_gain = hit;
        }
      }
    }
  }

  if (abs(guess[0] - last_guess [0]) < 0.001) {
    best_x = 0;
  } 
  if (abs(guess[1] - last_guess [1]) < 0.001) {
    best_y = 0;
  }



  if (best_x != (last_err[0] - robot->err_x) || best_y != (last_err[1] - robot->err_y) || c_col[0] > 0.01f || c_col[1] > 0.01f) {
    for (int j1 = 0; j1 < window; ++j1)
    {
      for (int j2 = 0; j2 < window; ++j2)
      {
        if (relative_map[j1][j2] == -1) {
          printf("_\t");
        } else {
          printf("%.3f\t", relative_map[j1][j2]);
        }
        
      }
      cout << endl;
    }

    cout << endl;

    for (int j1 = 0; j1 < window; ++j1)
    {
      for (int j2 = 0; j2 < window; ++j2)
      {

        int x_actual = x1 - window_shift + j1;
        int y_actual = y1 - window_shift + j2;
        if (tile_grid[x_actual][y_actual] == -1) {
          printf("_\t");
        } else {
          printf("%.3f\t", tile_grid[x_actual][y_actual]);
        }
      }
      cout << endl;
    }
    
    cout << endl;

    for (int j1 = 0; j1 < window; ++j1)
    {
      for (int j2 = 0; j2 < window; ++j2)
      {

        int x_actual = x1 - window_shift + best_x + j1;
        int y_actual = y1 - window_shift + best_y + j2;

        if (abs(relative_map[j1][j2] - tile_grid[x_actual][y_actual]) == 0) {
          printf("_\t");
        } else {
          printf("%.3f\t", abs(relative_map[j1][j2] - tile_grid[x_actual][y_actual]));
        }
      }
      cout << endl;
    }


    cout << endl;

    last_pos = {this_pos[0], this_pos[1]};
    this_pos = {robot->pos_x, robot->pos_y};
    cout << best_x << " " << best_y << endl;
    cout << last_guess[0] << " " << last_guess[1] << endl;
    cout << guess[0] << " " << guess[1] << endl;
    cout << (last_err[0] - robot->err_x) << " " << (last_err[1] - robot->err_y) << endl;
    cout << (guess[0] - last_guess [0]) << " | " << (guess[1] - last_guess [1]) << endl;
    
    cout << last_guess[0] << " " << last_guess[1] << endl;
    cout << robot->raw_x << " " << robot->raw_y << endl;
    cout << robot->pos_x << " " << robot->pos_y << endl;
    cout << "C " << c_col[0] << " " << c_col[1] << " " << (this_pos[0] - last_pos[0]) << " " << (this_pos[1] - last_pos[1]) <<  " " << robot->err_x << " " << robot->err_y << " "  << endl;
  }

  offset[0] += best_x * grid_div;
  offset[1] += best_y * grid_div;

  offset[0] = clamp((-10.0f * 0.2f) / 1.05f,  offset[0], (10.0f * 0.2f) / 1.05f);
  offset[1] = clamp((-10.0f * 0.2f) / 1.05f,  offset[1], (10.0f * 0.2f) / 1.05f);


  last_err = {robot->err_x, robot->err_y};

  last_guess = {robot->pos_x/1.05f + offset[0], robot->pos_y/1.05f + offset[1], robot->pos_t};

  if (abs(time_delta) > 1000) {
    guess_diff = {last_guess[0] - save_guess[0], last_guess[1] - save_guess[1], last_guess[2] - save_guess[2]};
    save_guess = {last_guess[0], last_guess[1], last_guess[2]};
  }

  // drive_shift = {guess[0] - (last_guess[0] + c_col[0]), guess[1] - (last_guess[1] + c_col[1])};
  
  // c++;
  // if (c > 100) {
  //   exit(0);
  // } 
}

int check_count = 0;
int check_limit = 25;
int check_count2 = 0;
int check_limit2 = 500;

std::deque<std::vector<int>> target_path;
std::vector<int> target_point = {};
bool has_path = false;

bool hit_drive = false;

bool drive_to_point (Robot* robot, std::vector<int> point) {
  draw_point(float(point[0] * grid_div), float(point[1] * grid_div), 1.0f, 0, 1.0f);
  float dx = (float(point[0] * grid_div) - grid_offset) - guess[0];
  float dy = (float(point[1] * grid_div) - grid_offset) - guess[1];
  float ang = atan2(dy, dx);
  float dist = dx * dx + dy * dy;
  float tang = ang - guess[2];
  if (tang > PI) {
    tang -= PI * 2;
  } else if (tang < -PI) {
    tang += PI * 2;
  }

  float turn = 1.5f;
  if (tang < -0.10){
    if (dist > 2 * 2 && tang > -0.3 && hit_drive) {
      cmd_vel(robot, 3.0, 3.0 - turn, false);
    } else {
      cmd_vel(robot, 3.0, -3.0, false);
    }
     if (robot->ranges[3].range < 0.4 || robot->ranges[2].range < 0.4 || robot->ranges[4].range < 0.4) {
      cout << "cancel " << robot->ranges[3].range << " " <<  robot->ranges[2].range << " " << robot->ranges[4].range << endl;
      return true;
    } 
    cout << "rotating " << ang << " " << guess[2]  << " " << tang << endl;
    return false;
  } else if (tang > 0.10){
    if (dist > 2 * 2 && tang < 0.3 && hit_drive) {
      cmd_vel(robot, 3.0 - turn, 3.0, false);
    } else {
      cmd_vel(robot, -3.0, 3.0, false);
    }
    cout << "rotating " << ang << " " << guess[2] << " " << tang << endl;
     if (robot->ranges[3].range < 0.4 || robot->ranges[2].range < 0.4 || robot->ranges[4].range < 0.4) {
      cout << "cancel " << robot->ranges[3].range << " " <<  robot->ranges[2].range << " " << robot->ranges[4].range << endl;
      return true;
    } 
    return false;
  } else if (dist > 0.5 * 0.5) {
    cout << "driving" << ang << " " << guess[2] << " " << tang << endl;
    hit_drive = true;
    if (robot->ranges[3].range < 0.4 || robot->ranges[2].range < 0.4 || robot->ranges[4].range < 0.4) {
      cout << "cancel " << robot->ranges[3].range << " " <<  robot->ranges[2].range << " " << robot->ranges[4].range << endl;
      return true;
    } 
    cmd_vel(robot, 3, 3, false);
    return false;
  }

  hit_drive = false;

  return true;
}

bool left_follow = false;
bool right_follow = false;

void main_drive(Robot* robot) {
  //cout << "validating" << endl;
  //bool result = validate_coords(dirty_input);
  //cout << "validated" << endl;
  guess = {robot->pos_x/1.05f + offset[0], robot->pos_y/1.05f + offset[1], robot->pos_t};
  cout << coll[0] << " = "<< coll[1] <<  " " << time_delta << endl;
  cout << save_guess[0] - guess[0] << " - "<< save_guess[1] - guess[1] <<  " " << time_delta << endl;
  fix_guess(robot);
  guess = {robot->pos_x/1.05f + offset[0], robot->pos_y/1.05f + offset[1], robot->pos_t};
  
  // if (check_count > check_limit && !result) {
  //   cmd_vel(robot, 0.0, 0.0, false);
  // } else {

  // if (check_count > 20) {
  //   cmd_vel(robot, 0, 0, false);
  //   check_count = 0;
  //   return;
  // }

  for(const auto hit: robot->ranges) {
    float r = hit.range;
    if (std::isinf(r)) {
      r = 2.0001f;
    }

    float x_val1 = guess[0] + clamp(0.0f, r, 2.0f) * cos(hit.angle + guess[2]);
    float y_val1 = guess[1] + clamp(0.0f, r, 2.0f) * sin(hit.angle + guess[2]);
    bresenham(guess[0], guess[1], x_val1, y_val1, r <= 2.0f, false, tile_grid, true);
  }

  if (check_count2 > check_limit2) {
    if (!has_path) {
      cout << " o " << endl;
      target_path = reduce_path(get_target(guess[0], guess[1]));

      if (target_path.size() > 0) {
        has_path = true;
        target_point = target_path.back();
        target_path.pop_back();
      } else {
        check_count2 = 0;
        has_path = false;
      }
    } else if (has_path) {
      float dx = float(target_point[0] * grid_div) - float(grid_offset);
      float dy = float(target_point[1] * grid_div) - float(grid_offset);
      cout << "driving to = [" << dx << ", " << dy << "] p_left: [" << target_path.size() << "] current = [" << guess[0] << " " <<  guess[1] << "]" << endl;
      if (tile_grid[target_point[0]][target_point[1]] > 0 ) {
        cout << "path_updated:" << tile_grid[target_point[0]][target_point[1]] << endl;
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

  cout << "cc:" << check_count2 << endl;
  check_count++;

  float brgt = clamp(0.0, robot->ranges[0].range, 2.0);
  float rgt = clamp(0.0, robot->ranges[1].range, 2.0);
  float frgt = clamp(0.0, robot->ranges[2].range, 2.0);
  float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
  float flft = clamp(0.0, robot->ranges[4].range, 2.0);
  float lft = clamp(0.0, robot->ranges[5].range, 2.0);
  float blft = clamp(0.0, robot->ranges[6].range, 2.0);
  
  float speed = 2.5f;
  float turn = 1.5f;
  float f_min = 0.6f;
  float f2_min = 0.75f;
  float f3_min = 0.9f;

  if (fwd < 1.2 || frgt < f_min || flft < f_min) {
    if (lft < 1.0 && flft > 1.8 || left_follow) {
      cout << "A1" << endl;
      cmd_vel(robot, -speed, speed, false);
    } else if (rgt < 1.0 && frgt > 1.8 || right_follow) {
      cout << "A2" << endl;
      cmd_vel(robot, speed, -speed, false);
    } else if (rgt + frgt > lft + flft) {
      cout << "B1" << endl;
      cmd_vel(robot, speed, -speed, false);
    } else {
      cout << "B2" << endl;
      cmd_vel(robot, -speed, speed, false);
    }
  } else if ( lft < 1.8 && lft > 1.1 && flft > lft ) {
    cout << "E" << endl;
    cmd_vel(robot, speed - turn, speed + turn, false);
  } else if ((flft / sqrt(2) < f2_min || lft < f2_min)) {
    if (flft / sqrt(2) < 0.6 && lft < flft / sqrt(2)) {
      cmd_vel(robot, speed, speed, false);
    } else if (lft / 1.3 > flft / sqrt(2) || flft / sqrt(2) < f_min) {
      cmd_vel(robot, speed + turn, speed - turn, false);
      cout << "F1" << endl;
    } else if (lft * 1.3 < flft / sqrt(2) && flft < f3_min) {
      cmd_vel(robot, speed - turn, speed + turn, false);
      cout << "F2" << endl;
    } else {
      cmd_vel(robot, speed, speed, false);
    }

    // left_follow = false;
    // right_follow = true;
  } else if (rgt < 1.8 && rgt > 1.1 && frgt > rgt && blft > 1.7 && lft > 1.7 && flft > 1.7) {
    cout << "C" << endl;
    cmd_vel(robot, speed + turn, speed - turn, false);
  } else if ((frgt / sqrt(2) < f2_min || rgt < f2_min) && blft > 1.7 && lft > 1.7 && flft > 1.7) {
    if (frgt / sqrt(2) < 0.6 && rgt < frgt / sqrt(2)) {
      cmd_vel(robot, speed, speed, false);
    } else if (rgt / 1.3 > frgt / sqrt(2) || frgt / sqrt(2) < f_min) {
      cmd_vel(robot, speed - turn, speed + turn, false); 
      cout << "D1" << endl;
    } else if (rgt * 1.3 < frgt / sqrt(2) && frgt < f3_min) {
      cmd_vel(robot, speed + turn, speed - turn, false);
      cout << "D2" << endl;
    } else {
      cmd_vel(robot, speed, speed, false);
    }
    // left_follow = true;
    // right_follow = false;
  } else if (((blft < 1.5 && (lft > 1.5 || flft > 1.5)) || (lft < 1.3 && flft > 1.5)) && rgt > 1.2) {
    cmd_vel(robot, speed - turn * 2, speed + turn, false);
    cout << "G" << endl;
  } else if (((brgt < 1.5 && (rgt > 1.5 || frgt > 1.5)) || (rgt < 1.3 && frgt > 1.5)) && lft > 1.2) {
    cmd_vel(robot, speed + turn, speed - turn * 2, false);
    cout << "G2" << endl;
  } else {
    cmd_vel(robot, speed, speed, false);
    cout << "fwd" << endl;
    if (rgt > 1.8 && frgt > 1.8) {
      right_follow = false;
    }
    if (lft > 1.8 && flft > 1.8) {
      left_follow = false;
    }
  }
}
int setup_c = 0;

void callback(Robot* robot) {
  // cheat_input = {robot->raw_x, robot->raw_y, robot->raw_t};
  // cheat_err_d = {robot->err_x - cheat_err[0], robot->err_y - cheat_err[1], robot->err_t - cheat_err[2]}; 
  // cheat_err = {robot->err_x, robot->err_y, robot->err_t}; 
  time_delta = robot->stamp - last_time;
  last_time = robot->stamp;
  // if (setup) {
  //   setup_stage(robot);
  // } else {
  if (setup_c < 20) {
    setup_c++;
    return;
  }
  main_drive(robot);
  // } 
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
