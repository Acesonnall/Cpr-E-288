/*
 * main.h
 *
 * Created: 4/16/2016 12:33:35 PM
 *  Author: Omar Taylor, Dalton Handel, Louis Hamilton
 */

/*! \file main.h
    \brief The control center.
*/

/// Moves the robot a specified distance. Written by Dalton and improved upon by Omar and Louis.
/**
* A recursive function that utilizes the oi_set_wheels function of open interface to move the object a certain distance.
* @param self a structure storing the iRobot Create's sensor data.
* @param distance_mm the distance the iRobot Create will travel in centimeters (to be converted to mm).
* @param obst a structure storing relevant information related to object detection and tracking.
* @param bot a structure keeping track of the robot's Cartesian coordinates and direction the robot is facing.
* @param c a structure storing relevant information related to manual operation of the robot. In this function, it allows the robot to play a specified song when it reaches the retrival zone.
*/
void move(oi_t *self, float distance_mm, obstacle* obst, robot* bot, control c);

/// Rotates the robot a specified angle. Written by Dalton.
/**
* A function that utilizes the oi_set_wheels function of open interface to rotate the object to a certain degree.
* @param self a structure storing the iRobot Create's sensor data.
* @param degrees the angle the iRobot Create will rotate in degrees. Positive degrees is counter-clockwise and negative degrees is clockwise.
* @param bot a structure keeping track of the robot's Cartesian coordinates and direction the robot is facing.
*/
void rotate(oi_t *self, float degrees, robot *bot);

/// Receives a command from the operator. Written by Omar.
/**
* A function that waits for a command from the operator and performs the corresponding action ('w' to move forward, 'a' to rotate left, 'd' to rotate right, 's' to indirectly move backwards, 'q' to scan, 'r' to reset tracked objects, 'b' to re-initialize the robot's Cartesian coordinates and angle, and '1' to play a song.
* @param c a structure storing relevant information related to manual operation of the robot. In this function, it allows the robot to operate based on input given by the operator via bluetooth communication.
* @param obst a structure storing relevant information related to object detection and tracking. Needs to be passed in to be used by other functions called within.
* @param self a structure storing the iRobot Create's sensor data. Needs to be passed in to be used by other functions called within.
* @param bot a structure keeping track of the robot's Cartesian coordinates and direction the robot is facing. Needs to be passed in to be used by other functions called within.
*/
void get_command(control c, obstacle* obst, oi_t *self, robot* bot);

/// Reads data from the robot's cliff sensors. Written by Omar.
/**
* A function that reads the robot's underhand infrared sensors and prints the data on the robot's LCD screen. Useful for calibration purposes.
* @param self a structure storing the iRobot Create's sensor data
*/
void read_cliff_sensors(oi_t *self);

/// Logs position of hazards undetected by the infrared and sonar sensor. Written by Louis
/**
* A function that assigns a coordinate position, initial detection angle, and initial detection distance to objects found by the robot's bumpers and underside infrared sensors.
* @param obst a structure storing relevant information related to object detection and tracking. In this function, it is needed to assign information found.
* @param bot a structure keeping track of the robot's Cartesian coordinates and direction the robot is facing. In this function, it is needed for calculation.
* @param bumper_cliff the side of the robot the object was detected on by the bumper or cliff sensors
* @param object the type of object that was detected
* @param dist the distance the robot traveled in total. Necessary for performing accurate calculations.
*/
void log_position(obstacle* obst, robot* bot, char bumper_cliff, char object, signed char dist);

/// Helper method for log_position. Written by Louis
/**
* This method performs the calculations and initial position assignments for log_position to reduce code redundancy.
* @param obst a structure storing relevant information related to object detection and tracking. In this function, it is needed to assign information found.
* @param bot a structure keeping track of the robot's Cartesian coordinates and direction the robot is facing. In this function, it is needed for calculation.
* @param dist the distance the robot traveled in total. Necessary for performing accurate calculations.
*/
void log_position_helper(obstacle* obst, robot* bot, signed char dist);