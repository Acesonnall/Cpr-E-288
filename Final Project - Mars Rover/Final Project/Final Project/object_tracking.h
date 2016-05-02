/*
 * object_tracking.h
 *
 * Created: 3/28/2016 1:03:47 PM
 *  Author: Omar Taylor, Dalton Handel, Louis Hamilton, Souparni Agnihotri
 */ 

/*! \file object_tracking.h
    \brief The file in which object tracking is handled.
*/

#include "open_interface.h"

/* Bluetooth Definitions */
/*! \def FOSC
    \brief Clock Speed
*/
#define FOSC 16000000
/*! \def BAUDBLUETOOTH
	\brief Bluetooth baud rate
*/
#define BAUDBLUETOOTH 57600
/*! \def UBRR
	\brief Bluetooth Connection
*/
#define UBRR FOSC/8/(BAUDBLUETOOTH-1)

/* Detection Range Definition */
/*! \def MAX_DETECTION_DISTANCE
	\brief maximum measurable gap (in cm)
*/
#define MAX_DETECTION_DISTANCE 50

/* Object Linear Width Definitions */
/*! \def SMALL_OBJECT_SIZE_MIN
	\brief Robot detects minimum linear width of smallest object as 3 cm
*/
#define SMALL_OBJECT_SIZE_MIN 3
/*! \def SMALL_OBJECT_SIZE_MAX
	\brief Robot detects maximum linear width of smallest object as 6 cm
*/
#define SMALL_OBJECT_SIZE_MAX 6
/*! \def MEDIUM_OBJECT_SIZE_MIN
	\brief Robot detects minimum linear width of medium object as 8 cm
*/
#define MEDIUM_OBJECT_SIZE_MIN 8
/*! \def MEDIUM_OBJECT_SIZE_MAX
	\brief Robot detects maximum linear width of medium object as 11 cm, but will use 10 cm for the variable. Better to undershoot.
*/
#define MEDIUM_OBJECT_SIZE_MAX 10
/*! \def LARGE_OBJECT_SIZE_MIN
	\brief Robot detects minimum linear width of largest object as 11 cm
*/
#define LARGE_OBJECT_SIZE_MIN 11
/*! \def LARGE_OBJECT_SIZE_MAX
	\brief Robot detects maximum linear width of largest object as 21 cm
*/
#define LARGE_OBJECT_SIZE_MAX 21

/* Object Array Indices */
/*! \def ALL_ANGULAR_WIDTH
	\brief Definition of the calculated angular width of the object for use in the objects array
*/
#define ALL_ANGULAR_WIDTH 0
/*! \def ALL_LINEAR_WIDTH
	\brief Definition of the calculated linear width of the object for use in the objects array
*/
#define ALL_LINEAR_WIDTH 1
/*! \def ALL_DISTANCE_SONAR
	\brief Definition of the detected SONAR distance from the robot's ping sensor (located on the servo) to the object for use in the objects array
*/
#define ALL_DISTANCE_SONAR 2
/*! \def ALL_DISTANCE_IR
	\brief Definition of the detected infra-red distance from the robot's infra-red sensor (located on the servo) to the object for use in the objects array
*/
#define ALL_DISTANCE_IR 3
/*! \def ALL_POSITION
	\brief Definition of the calculated angle an object was detected at for use in the objects array
*/
#define ALL_POSITION 4
/*! \def ALL_X
	\brief The calculated X coordinate of a detected object
*/
#define ALL_X 5
/*! \def ALL_Y
	\brief The calculated Y coordinate of a detected object
*/
#define ALL_Y 6

/* Definitions for Bumper and Cliff Sensors */
/*! \def LEFT
	\brief Definition of the left bumper sensor
*/
#define LEFT 1
/*! \def MIDDLE
	\brief Definition of the left and right bumper sensor
*/
#define MIDDLE 2
/*! \def RIGHT
	\brief Definition of the right bumper sensor
*/
#define RIGHT 3
/*! \def CLIFF
	\brief Definition of the cliff sensor
*/
#define CLIFF 100
/*! \def WHITE
	\brief Definition of the white tape linear width value (arbitrarily assigned)
*/
#define WHITE 105
/*! \def RED
	\brief Definition of the red paper linear width value (arbitrarily assigned)
*/
#define RED 110
/*! \def FLAT
	\brief Definition of the flat object linear width value (arbitrarily assigned)
*/
#define FLAT 115

//! Structure of detection variables. Written by Omar.
/*! This is a structure for defining the obstacle detection variables. All the properties of the obstacle(s) detected are recorded here. */
typedef struct {
	volatile float degrees; /*!< Updates the number of degrees turned by the servo. Initially set to zero. */
	
	volatile int cur_dist_IR; /*!< Finds and updates the IR distance from the robot to the object. Initially set to zero. */
	volatile int last_dist_IR; /*!< Finds and updates the last IR distance found of the object. Initially set to zero. */
	volatile int total_dist_IR; /*!< Total distance measured by the IR. Initially set to zero. */
	volatile char start_angle_IR; /*!< This is used to calculate the object angular size. This variable is used to record the start angle at which the object was found. Initially set to zero. */
	volatile char end_angle_IR; /*!< This is used to calculate the object angular size. This variable is used to record the last angle at which the object was found. Initially set to zero. */
	volatile char object_detected : 1; /*!< Toggles when object has started and finished being detected. Can only process one object at a time. */
	
	volatile float cur_dist_SONAR; /*!< Finds and updates the SONAR distance from the robot to the object. Initially set to zero. */
	volatile float last_dist_SONAR; /*!< Finds and updates the last IR distance found of the object. Initially set to zero. */
	volatile char start_dist_SONAR; /*!< Records the first distance at which the object is found at by the SONAR. Initially set to zero. */
	volatile char end_dist_SONAR; /*!< Records the last distance at which the object is found at  by the SONAR. Initially set to zero. */
	
	volatile char smallest_obj_angular_size : 6; /*!< Finds and records the value of the angular size of the smallest object found. Initially set to 50. */
	volatile char smallest_obj_linear_size : 5; /*!< Finds and records the value of the angular size of the smallest object found. Initially set to 11. */
	volatile char smallest_obj_dist_SONAR; /*!< Finds and records the SONAR distance of the smallest object found. Initially set to zero. */
	volatile char smallest_obj_dist_IR; /*!< Finds and records the IR distance of the smallest object found. Initially set to zero. */
	volatile float smallest_obj_position; /*!< Finds and records the angular position of the smallest object. Initially set to zero. */
	
	volatile char closest_obj_angular_size : 6; /*!< Finds and records the value of the angular size of the closest object. Initially set to zero. */
	volatile char closest_obj_linear_size : 5; /*!< Finds and records the linear size of the closest object. Initially set to zero. */
	volatile int closest_obj_dist_SONAR; /*!< Finds and records the distance of the closest object using SONAR. Initially set to 341. */
	volatile int closest_obj_dist_IR; /*!< Finds and records the distance of the closest object using IR. Initially set to 2752. */
	volatile float closest_obj_position; /*!< Finds and records the angular position of the closest object. Initially set to zero. */
	
	volatile char validation_level; /*!< This is the validation level variable which gets updated every time an object is detected. It is used to check for anomalies. */
	
	volatile float all_objects_array[30][7]; /*!< Storing all the objects found. There are 15 objects in total (30 to account for cliffs and bumper-detected objects), each with 9 parameters (Angular width, linear width, distance_sonar, distance_ir, angular position in respect to the bot, and, x & y coordinate).*/
	volatile char all_object_index : 5;  /*!< Keeps track of the index of every object found.*/
	
	
} obstacle;

//! Structure of robot variables. Written by Souparni and improved upon by Omar.
/*! This is a structure for defining the robot's variables based on the idea of a Cartesian coordinate system. */
typedef struct  
{
	float x; /*!< Defines and records the x coordinate position of the robot. Initially set to 0. */ 
	float y; /*!< Defines and records the y coordinate position of the robot. Initially set to 0. */  
	float angle; /*!< Defines and records the angle of the robot. Initially set to 90. */
	float dist_traveled; /*!< Defines and records the total distance traveled by the robot. Initially set to zero. Reset after every movement. */
	char initialized : 1; /*!< Checks if the robot is initialized. Returns 1 or 0 (True or false) */
	
} robot;

//! Structure of command variables. Written by Omar.
/*! This is a structure for defining the robot's command variables. Used for loading songs and allowing the user to manually operate the robot. */
typedef struct {
	char user_command; /*!< Calls on the user to give the command. Initially at 0. */
	
	char travel_dist : 4; /*!< Specific distance to travel by the robot. Initially set at 15 cm. */
	char angle_to_turn : 6; /*!< Specific angle to turn by the robot. Initially set at 45 degrees. */
	
	unsigned char s1_id : 1;  /*!< The id number of the song to be played. */
	unsigned char s1_num_notes : 2;  /*!< The number of notes for the song to be played. */
	unsigned char s1_notes[3]; /*!< Array of notes of the song */
	unsigned char s1_duration[3];  /*!< Time duration of the song. */
	
	unsigned char s2_id : 2; /*!< The id number of the song to be played. */
	unsigned char s2_num_notes : 7;  /*!< The number of notes for the song to be played. */
	unsigned char s2_notes[96];  /*!< Array of notes of the song */
	unsigned char s2_duration[96];  /*!< Time duration of the song. */
	
} control;

/// Prepares LCD, IR, SONAR, Servo, USART, and object detection structure. Written by Omar.
/**
* Initializes the obstacles and robot variables and also prepares LCD, IR, SONAR, Servo, USART, and object detection structure.
* @param obst the pointer used to refer to the variables in the obstacle struct, specifically initializing all the variables accordingly.
* @param bot the pointer used to refer to the variables in the robot struct. Robot is initialized to (0, 0) and set to 90 degrees once unless re-initailzed manually. 
* @param c the pointer used to refer to the variables in the control struct. Loads songs onto the robot to be used later.
*/
void initalizations(obstacle* obst, robot* bot, control* c);

/// Performs a sweep to detect the closest objects. Written by Omar.
/**
* Perform 180 degree sweep, finding the smallest and closest object in the process.
* @param obst the pointer used to refer to the variables in the obstacle struct. Specifically the cur_dist_IR, and the cur_dist_SONAR variables that are updated constantly.
* @param bot the pointer used to refer to the variables in the robot struct. The bot variables are being updated by calling other methods inside this method.
*/
void sweep(obstacle* obst, robot* bot);

/// Finds the linear width of the object detected. Written by Dalton and improved upon by Omar.
/**
* Finds the linear width of a detected object, after finding object's distance and angular width first. [sin((pi/180) * theta) * hypotenuse].
* @param obst the pointer used to refer to the variables in the obstacle struct.
* @return the linear width of the object
*/
char get_linear_width(obstacle* obst);

/// Updates the information of detected obstacles and the robot. Written by Omar and Louis.
/**
* Updates the information of detected obstacles and the robot based on equations related to a Cartesian coordinate system.
* @param obst the pointer used to refer to the variables in the obstacle struct. Necessary to update the distance and direction of the logged obstacles in relation to the robot.
* @param bot the pointer used to refer to the variables in the robot struct. Necessary to update the direction the robot is facing as well as its coordinates.
*/
void update_information(obstacle* obst, robot* bot);

/// Finds the smallest object out of the found objects. Written by Omar.
/**
* Finds the smallest object out of the found objects by comparing the linear width of each found object with a predefined width of the smallest object. It then sets the width of the smallest object.
* @param obst the pointer used to refer to the variables in the obstacle struct. Here, the specific values (like width and angle it was found at) of the smallest objects are updated.
*/
void find_smallest_obj(obstacle* obst);

/// Find any objects and log their stats in the object array. Written by Omar.
/**
* Logs the angle and the distance of every object detected and stores it into an all_objects_array array. This method can be called in order to find any specific object.
* @param obst the pointer used to refer to the variables in the obstacle struct. All the angles and distances of the obstacles are updated.
* @param bot the pointer used to refer to the variables in the robot struct.
*/
void find_objs_IR(obstacle* obst, robot* bot);

/// Finds the closest object out of the found objects. Written by Omar.
/**
* Finds the closest object out of the found objects by comparing the linear width of each found object with a predefined width of the closest object. It then sets the width of the closest object.
* @param obst the pointer used to refer to the variables in the obstacle struct. Here, the specific values (like width and angle it was found at) of the closest objects are updated.
*/
void find_closest_obj(obstacle* obst);

/// Show and tell, baby. Written by Omar.
/**
* Generic function that prints out information found by the robot into a readable format over bluetooth.
* @param obst Uses stats recorded in this struct to create readable statistics.
*/
void print_and_process_stats(obstacle* obst);

/// Finding the duplicate object. Written by Louis.
/**
* If there is a duplicate object found, it is not logged. This method makes sure the same object is not logged twice.
* @param obst the pointer used to refer to the variables in the obstacle struct. A specific object is passed and checked if it is a duplicate.
* @param bot the pointer used to refer to the variables in the robot struct.
*/
void find_dupilicate(obstacle* obst, robot* bot);

/// Resets the object array. Written by Omar.
/**
* We are setting the index of the object detected to zero, and also clearing all the objects in the array.
* @param obst the pointer used to refer to the variables in the obstacle struct. Here, each object is cleared and reset.
*/
void reset_object_array(obstacle* obst);

/// Resets the robot. Written by Omar.
/**
* We are setting the x and y coordinate and the angle and distance traveled to zero.
* @param bot the robot to be reset.
*/
void reinitialize_bot(robot* bot);