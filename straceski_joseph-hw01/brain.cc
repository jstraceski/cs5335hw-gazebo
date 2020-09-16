
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;
using std::min;
using std::max;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;
// sum of hit angles near the fron of the bot to trigger a turn
double densityThresholdVal = 3.14 * 3;
// factor applied to the damping factor, ideally this times the damping max
//   should be less than 1 
double damping = 0.08;
// turning history accumulator, this value is added to every time the robot
//   issues a turn command and causes repeated turns to have diminishing returns
double turnAcc = 0;
// speed of the bot (there is only one, fast)
double speed = 20;
// scalar applied every tick to the turnAcc value this will cause it to natually
//   return to 0 if no turns are happening
double dampingReduction = 0.99;
// max angle relative to the front of the robot that triggers a turn
double maxAng = 3.14/1.75;
// distance from the robot that triggers a valid hit detection
double range = 1.5;
// minim radian range +- that triggers the bot to attempt to correct its 
//   headding twoards the goal area
double defaultTurnRad = 0.01;
// max value +- of the turnAcc
double dampingMax = 10;
// turning speed of the robot
double turnSpeed = 0.5;
// factor applied to the turning speed when it is being added to the
//   turnAcc (this simulates a time delta)
double accFactor = 0.5;

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

    double lDensity = 0;
    double rDensity = 0;

    for (LaserHit hit : robot->hits) {     
        if (hit.range < range) {
            if (hit.angle > 0 && hit.angle < maxAng) {
                // if the robot detects a hit on the left side add it to the 
                //   left hit density accumulator
                lDensity += (maxAng - hit.angle);
            } else if (hit.angle <= 0 && hit.angle > -maxAng) {
                // identical to the left accumulator but for the right side
                rDensity += (maxAng + hit.angle);
            }
        }
    }
  
    // set speed and clear any previous turns
    robot->set_vel(speed);
    robot->set_turn(0);
    // scale the turn accumulator by the reduction factor
    turnAcc *= dampingReduction;
    
    // if the densities are above the threshold turn to avoid the obstacle
    if (leftDensity + rightDensity > densityThresholdVal) {
        if (leftDensity > rightDensity) {
            robot->set_turn(0.5);
            turnAcc = 0; // reset the turn acc to allow for sharp corrections
        } else {
            robot->set_turn(-0.5);
            turnAcc = 0;
        }
    } else {
        // the angle of the robot relative to the goal in the global space
        float angle = atan2(dy, dx);
        // limit the turn accumulator
        turnAcc = max(min(turnAcc, dampingMax), -dampingMax);
        if (angle < -defaultTurnRad) {
            // set the turn speed offset by damping from the turn accumulator
            robot->set_turn(turnSpeed * (1 - (damping * max(turnAcc, 0.0))));
            turnAcc += turnSpeed * accFactor;
        } else if (angle > defaultTurnRad) {
            robot->set_turn(-turnSpeed * (1 + (damping * min(turnAcc, 0.0))));
            turnAcc -= turnSpeed * accFactor;
        }
    }
    // display debug information
    cout << "l = " << leftDensity << ", r = " << rightDensity << " s = " << turnAcc << endl;
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
