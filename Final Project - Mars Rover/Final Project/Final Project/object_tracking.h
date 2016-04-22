/*
 * object_tracking.h
 *
 * Created: 3/28/2016 1:03:47 PM
 *  Author: Omar Taylor
 */ 

#include "open_interface.h"

/* USART Bluetooth Definitions */
#define FOSC 16000000 // Clock Speed
#define BAUDBLUETOOTH 57600 // Bluetooth baud rate
#define UBRR FOSC/8/(BAUDBLUETOOTH-1) // Bluetooth Connection

/* Measurable Gap */
#define MAX_DETECTION_DISTANCE 50 // cm
#define MIN_DETECTION_DISTANCE 10 // cm

/* Object Linear Widths */
#define SMALL_OBJECT_SIZE_MIN 3   // Robot detects minimum linear width of smallest object as 3 cm
#define SMALL_OBJECT_SIZE_MAX 6   // Robot detects maximum linear width of smallest object as 6 cm
#define MEDIUM_OBJECT_SIZE_MIN 8  // Robot detects minimum linear width of medium object as 8 cm
#define MEDIUM_OBJECT_SIZE_MAX 10 // Robot detects maximum linear width of medium object as 11 cm, but will use 10 cm for the variable. Better to undershoot.
#define LARGE_OBJECT_SIZE_MIN 11  // Robot detects minimum linear width of largest object as 11 cm
#define LARGE_OBJECT_SIZE_MAX 21  // Robot detects maximum linear width of largest object as 21 cm

/* Definitions for All Objects Array Column Indices */
#define ALL_ANGULAR_WIDTH 0
#define ALL_LINEAR_WIDTH 1
#define ALL_DISTANCE_SONAR 2
#define ALL_DISTANCE_IR 3
#define ALL_POSITION 4
#define ALL_X 5
#define ALL_Y 6

/* Definitions for Cliff & Bumper Sensors */
#define LEFT 1
#define MIDDLE 2
#define RIGHT 3
#define CLIFF 100
#define WHITE 105
#define RED 110
#define FLAT 115

/* Definitions for Goal Post Array Column Indices
#define GOALP_DISTANCE 0
#define GOALP_POSITION 1

Definitions for Obstacle Array Column Indices
#define OBST_DISTANCE 0
#define OBST_POSITION 1

Compass Definitions for Object Arrays
#define EAST 2
#define NORTH_EAST 3
#define NORTH 4
#define NORTH_WEST 5
#define WEST 6*/

/* Detection Variables */
typedef struct {
	volatile float degrees;
	
	/* IR Variables */
	volatile int cur_dist_IR;
	volatile int last_dist_IR;
	volatile int total_dist_IR;
	volatile char start_angle_IR;
	volatile char end_angle_IR;
	volatile char object_detected : 1;               // Can only process one object at a time
	volatile char cur_obj_size_IR : 5;               // Max object size is 17 cm
	
	/* SONAR Variables */
	volatile float cur_dist_SONAR;
	volatile float last_dist_SONAR;
	volatile char start_dist_SONAR;
	volatile char end_dist_SONAR;
	
	/* Smallest Object Variables */
	volatile char smallest_obj_angular_size : 6;
	volatile char smallest_obj_linear_size : 5; // Smallest possible object size is the max possible object size
	volatile char smallest_obj_dist_SONAR;
	volatile char smallest_obj_dist_IR;
	volatile float smallest_obj_position;
	
	/* Closest Object Variables */
	volatile char closest_obj_angular_size : 6; // Smallest possible object angle size is within 6 bits
	volatile char closest_obj_linear_size : 5; // Smallest possible object size is the max possible object size (17 for linear)
	volatile int closest_obj_dist_SONAR;
	volatile int closest_obj_dist_IR;
	volatile float closest_obj_position;
	
	/* Object Validation Level Variable */
	volatile char validation_level; // Validation level probably won't exceed 5 bits.
	
	/* Goal Post Variables to Analyze | Used in ai.c */
	volatile float goal_post_array[4][2]; // Each column stores information about the goal posts (4 total) distance, angular position, and compass direction
	volatile char goal_post_index : 3; // Amount of goal posts found. There are 4 posts total so 3 bits will suffice.
	
	/* Obstacle Variables to Analyze | Used in ai.c
	volatile char obst_array[11][7]; // Each column stores information about the objects (11 total) distance, angular position, and compass direction
	volatile char obst_index : 3; // Amount of goal posts found. There are 4 posts total so 3 bits will suffice.*/
	
	/*Object Array Variables */
	volatile float all_objects_array[30][7]; // 15 objects total (30 to account for cliffs and bumper-detected objects), each with 9 parameters (Angular width, linear width, distance_sonar, distance_ir, angular position in respect to the bot, x & y coordinate)
	volatile char all_object_index : 4;                  // Max amount of objects is ~14
	
	
} obstacle;

typedef struct  
{
	float x;
	float y;
	float angle;
	float dist_traveled; // cm
	char initialized : 1; // True or False
	
} robot;

typedef struct {
	char user_command;
	
	char travel_dist : 4;
	char angle_to_turn : 6;
	
	/* Song 1 */
	unsigned char s1_id : 1;
	unsigned char s1_num_notes : 2;
	unsigned char s1_notes[3];
	unsigned char s1_duration[3];
	
	/* Song 2 */
	unsigned char s2_id : 2;
	unsigned char s2_num_notes : 7;
	unsigned char s2_notes[96];
	unsigned char s2_duration[96];
	
} control;

/************************************************************************/
/* Prepares LCD, IR, SONAR, Servo, USART, and object detection          */
/* structure                                                            */
/************************************************************************/
void initializations(obstacle* obst, robot* bot, control* c);

/************************************************************************/
/* Perform 180 degree sweep, finding the smallest and closest object in */
/* the process                                                          */
/************************************************************************/
void sweep(obstacle* obst, robot* bot);

/************************************************************************/
/* Finds the linear width of a detected object. Must have found         */
/* object's distance first. [sin((pi/180) * theta) * hypotenuse]        */
/************************************************************************/
char get_linear_width(obstacle* obst);

/************************************************************************/
/*                                                                      */
/************************************************************************/
void update_information(obstacle* obst, robot* bot, oi_t* self);

/************************************************************************/
/* Re-initialize everything and clear the object array                  */
/************************************************************************/
void reset(obstacle* obst, robot* bot, control* c);

/************************************************************************/
/* Find the smallest object out of the found objects                    */
/************************************************************************/
void find_smallest_obj(obstacle* obst);

/************************************************************************/
/* Find the any objects and log their stats in the object array         */
/************************************************************************/
void find_objs_IR(obstacle* obst, robot* bot);

/***********************************************************************
 Finds and categorizes objects in obst->object_array                  
**********************************************************************
void analyze_found_objects(obstacle* obst);*/

/************************************************************************/
/* Find the closest object out of the found objects                     */
/************************************************************************/
void find_closest_obj(obstacle* obst);

/************************************************************************/
/* Show and tell, baby                                                  */
/************************************************************************/
void print_and_process_stats(obstacle* obst);

void find_dupilicate(obstacle* obst, robot* bot);

void reset_object_array(obstacle* obst);

void reinitialize_bot(robot* bot);