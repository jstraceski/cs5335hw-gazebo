/*
A simple example of using the gfx library.
CSE 20211
9/7/2011
by Prof. Thain

Modified by Nat Tuck, Oct 2020
https://github.com/NatTuck/scratch-2020-09/tree/master/5335/08/hw06/brain

Modified by Sarah Coffen, Oct 2020
*/

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <mutex>
#include <iostream>
#include "robot.hh"

extern "C" {
#include "gfx.h"
}

int prev_robot_loc_x = -1;
int prev_robot_loc_y = -1;

using namespace std;
typedef lock_guard<mutex> guard;

mutex mx;
static int viz_init = 0;

int viz_hit(long xx, long yy, double color_val) {
	// guard _gg(mx);
	if (!viz_init) {
		puts("viz skip");
		return 0;
	}

	if (prev_robot_loc_x != -1 && prev_robot_loc_y != -1) {
		gfx_color(0, 100, 100);
		gfx_point(prev_robot_loc_x, prev_robot_loc_y);
		gfx_point(prev_robot_loc_x, prev_robot_loc_y + 1);
		gfx_point(prev_robot_loc_x + 1, prev_robot_loc_y);
		gfx_point(prev_robot_loc_x, prev_robot_loc_y - 1);
		gfx_point(prev_robot_loc_x - 1, prev_robot_loc_y);

		gfx_flush();
	}

	// puts("viz hit");

	int ww = 700;
	int hh = 700;
	int dd = min(ww, hh) / 2;

	int yy_fin = dd + (-1 * xx);
	int xx_fin = dd + (-1 * yy);

	// cout << xx_fin << " " << yy_fin << endl;

	int color_mod = int(color_val * 30); // -150 to 150
	if (color_val > 0) {
		gfx_color(200, 0, 100);
	} else {
		gfx_color(150 + color_mod, 100, 100);
	}

	gfx_point(xx_fin, yy_fin);

	gfx_flush();

	return 0;
}

void draw_robot_pose(long xx, long yy, double theta) {
	gfx_color(0, 200, 0);

	int ww = 700;
	int hh = 700;
	int dd = min(ww, hh) / 2;

	int yy_fin = dd + (-1 * xx);
	int xx_fin = dd + (-1 * yy);

	prev_robot_loc_x = xx_fin;
	prev_robot_loc_y = yy_fin;

	gfx_point(xx_fin, yy_fin);
	gfx_point(xx_fin, yy_fin + 1);
	gfx_point(xx_fin + 1, yy_fin);
	gfx_point(xx_fin, yy_fin - 1);
	gfx_point(xx_fin - 1, yy_fin);

	gfx_flush();
}

void viz_run(int argc, char **argv)
{
	int ysize = 700;
	int xsize = 700;

	char c;

	// gfx_clear_color(240,240,250);
	// Open a new window for drawing.
	gfx_open(xsize,ysize,"MAP");

	// Set the current drawing color to green.
	gfx_color(0,200,100);

	// // Draw a triangle on the screen.
	// gfx_line(100,100,200,100);
	// gfx_line(200,100,150,150);
	// gfx_line(150,150,100,100);
	usleep(50000);
	gfx_clear();

	{
	// guard _gg(mx);
	viz_init = 1;
	}

	while(1) {
		// Wait for the user to press a character.
		// c = gfx_poll();
		// cout << "WTF" << endl;
		// Quit if it is the letter q.
		if(c=='q') break;
		usleep(50000);
	}
}