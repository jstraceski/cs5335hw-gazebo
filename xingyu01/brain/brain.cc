
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

void
callback(Robot* robot)
{
    double rng = robot->range;
    cout << "range = " << rng << endl;

    if (robot->range < 1.0) {
        robot->set_vel(-2.5, 5.0);
        return;
    }

    if (robot->range < 1.7) {
        robot->set_vel(5.0, 5.0);
        return;
    }

    robot->set_vel(5.0, -2.5);
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
