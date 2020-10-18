
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

void
callback(Robot* robot)
{
    /*
    cout << endl;
    cout << "robot x =" << robot->pos_x << endl;
    cout << "robot y =" << robot->pos_y << endl;
    */
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    bool turn = false;

    for (LaserHit hit : robot->hits) {
        if (hit.range < 1.5) {
            if (hit.angle < 0.5 || hit.angle > (6.2 - 0.5)) {
                turn = true;
            }
        }
    }

    if (turn) {
        robot->set_vel(3.0);
        robot->set_turn(0.5);
    }
    else {
        robot->set_vel(5.0);
        robot->set_turn(-0.02);
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
