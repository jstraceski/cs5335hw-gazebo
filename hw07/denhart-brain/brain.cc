
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <queue>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;
using std::vector;
using std::queue;

// constants
const int ROW_SIZE = 80;
const int COL_SIZE = 80;
const float OCCUPIED_WEIGHT = 0.9;
const float FREE_WEIGHT = -0.7;

// data structures
enum Turn { left, right, straight, target };

struct Posn {
	float x;
	float y;
};

struct Cell { 
	int x; 
	int y; 
	bool hit; 
};

// override equality between cells
bool operator==(const Cell& c1, const Cell& c2)
{
	return c1.x == c2.x && c1.y == c2.y && c1.hit == c2.hit;
}

// state variables
struct State {
	vector<vector<float>> grid;
	vector<Cell> cells_to_check;
} state;

// converts the 'pos_t' param to a typical unit circle metric by
// converting it from a +/- pi range to a 0-2pi range
// and positively rotating it by pi/2 (so that 0 points west)
float
simple_t(float t)
{
	float new_range_t = (t < 0) ? (2 * M_PI + t) : t;
	float rotated_t = fmod((new_range_t + M_PI / 2), (2 * M_PI));
	return rotated_t;
}

// converts a raw x, y posn to being in range 0 - 80
// with the bottom left of the grid being 0, 0 and
// the top right being 80, 80
Posn
simple_posn(float x, float y)
{
	return { x + 40, y + 40 };
}

// converts a Posn to Cell in the occupancy grid
Cell
posn_to_cell(Posn posn, bool is_hit)
{
	return { floor(posn.x), floor(posn.y), is_hit };
}

// remove the given cell from cells to check if it exists
void
filter_cells_to_check(Posn posn)
{
	Cell cell = posn_to_cell(posn, false);
	while (find(state.cells_to_check.begin(),
							state.cells_to_check.end(),
							cell) != state.cells_to_check.end())
	{
		state.cells_to_check.erase(
				find(state.cells_to_check.begin(),
						 state.cells_to_check.end(),
						 cell));
	}
}

// sort cells to check based on proximity to robot
void
sort_cells_to_check(Robot* robot)
{
	Posn r_posn = simple_posn(robot->pos_x, robot->pos_y);

	/*
	// compare two cells based on their proximity from the robot
	bool compare_cells = [&] (Cell c1, Cell c2) -> bool { 
		float d1 = abs(r_posn.x - c1.x) + abs(r_posn.y - c1.y);
		float d2 = abs(r_posn.x - c2.x) + abs(r_posn.y - c2.y);
		return (d1 < d2);
	};

	sort(state.cells_to_check.begin(), 
			 state.cells_to_check.end(),
			 compare_cells);
	*/
}

// initializes the global occupancy grid
void
init_state()
{
	// initialize grid
	for (int y = 0; y < ROW_SIZE; y++)
	{
		vector<float> row;
		for (int x = 0; x < COL_SIZE; x++)
		{
			// add float to cell in grid
			row.push_back(0.0);

			// add cell to 'cell to check' list
			state.cells_to_check.push_back({ x, y, false });
		}
		
		state.grid.push_back(row);
	}
}

// calculates the position of the 'end' coordinates for a hit
// and returns all points (on an interval) between the current
// pos and the calculated end.
vector<Cell>
hit_to_cells(Robot* robot, auto hit, bool has_hit)
{
	// store relevant path information
	float t = simple_t(hit.angle); // is angle relative to heading?
	Posn posn = simple_posn(robot->pos_x, robot->pos_y);
  float dx = hit.range * sin(t);
	float dy = hit.range * cos(t);
	float interval = 0.1;

	// store each unique cell on the path from the robot to hit
	vector<Cell> cells;
	for (float i = 0.0; i < 1.0; i += interval)
	{
		Posn i_posn = { posn.x + dx * i, posn.y + dy * i };
		Cell cell = posn_to_cell(i_posn, false);
		if (find(cells.begin(), cells.end(), cell) == cells.end())
			cells.push_back(cell);
	}

	// assign the last cell stored to being hit
	cells.back().hit = has_hit;

	return cells;
}

// update the cell with new hit information
void 
update_cell(Cell cell)
{
	float update_value = cell.hit ? OCCUPIED_WEIGHT : FREE_WEIGHT;
	state.grid[cell.y][cell.x] += update_value;
}

// upgrades probabilities in the occupancy for a given hit
void 
update_grid(Robot* robot, auto hit, bool has_hit)
{
	// update grid state
	vector<Cell> cells = hit_to_cells(robot, hit, has_hit);
	for (Cell cell : cells)
		update_cell(cell);

	
	//cout << hit.range << "@" << hit.angle << endl;
}

// move the robot proportional to its distance from a wall
void move_robot(Robot* robot)
{	
	if (robot->ranges.size() < 5)
		return;

	float lft = clamp(0.0, robot->ranges[2].range, 2.0);
	float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
	float rgt = clamp(0.0, robot->ranges[4].range, 2.0);
	cout << "lft,fwd,rgt = "
			 << lft << ","
			 << fwd << ","
			 << rgt << endl;

	float spd = fwd * 3;
	float trn = clamp(-2.0, lft - rgt, 2.0);

	if (fwd < 1.5)
	{
		spd = 0;
		trn = 2;
	}
	
	cout << "spd,trn = " << spd << "," << trn << endl;
	robot->set_vel(spd + trn, spd - trn);
}

// wall following strategy
void
wall_follow(Robot* robot)
{
	if (robot->ranges.size() < 5)
		return;


}

void
display_grid()
{
	bool clear = true;
	for (int y = 0; y < state.grid.size(); y++)
	{
		vector<float> row = state.grid[y];
		for (float x = 0; x < row.size(); x++)
		{
			float val = row[x];
			if (val > 0)
			{
				cout << "val, x, y = "
						 << val << ", "
						 << x << ", "
						 << y << ", " << endl;
				float pct_x = x / (ROW_SIZE * 1.0);
				float pct_y = y / (COL_SIZE * 1.0);
				viz_hit(pct_x, pct_y, clear);
				clear = false;
			}
		}
	}
}

void
print_grid()
{
	for (vector<float> row : state.grid)
	{
		for (float weight : row)
		{
			cout << weight << " ";
		}
		cout << "\n" << endl;
	}
}

void
callback(Robot* robot)
{
	cout << "\n===" << endl;
	
	Posn cur_posn = simple_posn(robot->pos_x, robot->pos_y);	
	float cur_t = simple_t(robot->pos_t);
	cout << "x,y,t = "
			 << cur_posn.x << ","
			 << cur_posn.y << ","
			 << cur_t << endl;

	// remove a cell from cells to check if currently in a new cell
	// filter_cells_to_check(cur_posn);		

	// sort cells to check
	// sort_cells_to_check(robot);
	
	// update grid state
	for (auto hit : robot->ranges) {
		if (hit.range < 2) 
			update_grid(robot, hit, true);
	}

	// visualize the grid
	display_grid();

	// update robot position
	move_robot(robot);
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

int
main(int argc, char* argv[])
{
	cout << "initializing grid ....." << endl;
	init_state();
	cout << "making robot ....." << endl;
	Robot robot(argc, argv, callback);
	std::thread rthr(robot_thread, &robot);

	return viz_run(argc, argv);
}
