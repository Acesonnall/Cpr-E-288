/*
 * object_tracking.c
 *
 * Created: 3/28/2016 1:04:20 PM
 *  Author: Omar Taylor
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include "lcd.h"
#include "util.h"
#include "object_tracking.h"

void initializations(obstacle* obst, robot* bot) {
	obst->degrees = 0.0; // Start angle at 0
	
	lcd_init();               // Initialize LCD
	ADC_init();               // Initialize IR
	ping_timer_init();        // Initialize Ping
	servo_timer_init();       // Initialize Servo
	USART_Init(UBRR);         // Initialize USART for Bluetooth communication
	move_servo(&obst->degrees);     // Move Servo to starting position
	wait_ms(500);             // Wait for Servo to settle
	
	/* IR Variable Initializations */
	obst->cur_dist_IR = 0;
	obst->last_dist_IR = 0;
	obst->total_dist_IR = 0;
	obst->start_angle_IR = 0;
	obst->end_angle_IR = 0;
	obst->object_detected = 0; // Using ^= to toggle breaks program if used a lot. Why?
	obst->all_object_index = 0;
	obst->cur_obj_size_IR = 20;
	
	/* SONAR Variable Initializations */
	obst->cur_dist_SONAR = 0.0;
	obst->last_dist_SONAR = 0.0;
	obst->start_dist_SONAR = 0;
	obst->end_dist_SONAR = 0;
	
	/* Smallest Object Variable Initializations */
	obst->smallest_obj_angular_size = 50;
	obst->smallest_obj_linear_size = LARGE_OBJECT_SIZE_MIN;
	obst->smallest_obj_dist_SONAR = 0;
	obst->smallest_obj_dist_IR = 0;
	obst->smallest_obj_position = 0.0;
	
	/* Closest Object Variable Initializations */
	obst->closest_obj_angular_size = 0;
	obst->closest_obj_linear_size = 0;
	obst->closest_obj_dist_SONAR = 341; // Approximate SONAR Range Limit
	obst->closest_obj_dist_IR = 2752;   // Approximate IR Range Limit
	obst->closest_obj_position = 0.0;
	
	/* Object Validation Level Initialization */
	obst->validation_level = 0;
	
	/* Robot Coordinates Initialization */
	if (bot->initialized == 0) {
		bot->x = 0;
		bot->y = 0;
		bot->angle = 90.0;
		bot->dist_traveled = 0;
		bot->initialized ^= 1;
	}
	
	/* Goal Post Variables to Analyze
	obst->goal_post_index = 0; // Amount of goal posts found. There are 4 posts total so 3 bits will suffice.
	
	Obstacle Variables to Analyze
	obst->obst_index = 0; // Amount of goal posts found. There are 4 posts total so 3 bits will suffice.*/
	
	// Note: Object array does not need to be initialized
}

void sweep(obstacle* obst, robot* bot) {
	/* Clear PuTTY view and initialize columns */
	send_message("\f");
	send_message("Degrees       IR Distance (cm)    Sonar Distance (cm)\r\n");
	
	/* Data to be Sent to Putty */
	char buffer[53];
	
	/* Perform 180 degree scan. Collect distance measurements every 1 degree. */
	for (char i = 0; i <= 180; i++) {
		send_pulse();                                // Ping the SONAR sensor
		obst->cur_dist_IR = read_IR_distance();      // Get current IR distance measurement
		obst->cur_dist_SONAR = read_PING_distance(); // Get current SONAR distance measurement
		
		/* Prepare buffer for transmission */
		sprintf(buffer, "%-3d             %-4d                 %-3.4f\r\n", i, obst->cur_dist_IR, obst->cur_dist_SONAR);
		
		/* Send data to putty */
		send_message(buffer);
		
		/* Find Objects IR */
		find_objs_IR(obst, bot);
		
		obst->degrees++;            // Increment degree by 1
		move_servo(&obst->degrees); // Move servo into next position
		wait_ms(10);          // Wait for servo to position itself
	}
	
	/* Analyze Found Objects */
	//analyze_found_objects(obst);
	
	/* Find Smallest Object */
	find_smallest_obj(obst);
	
	/* Find Closest Object */
	find_closest_obj(obst);
}

char get_linear_width(obstacle* obst) {
	return 2 * obst->all_objects_array[obst->all_object_index][ALL_DISTANCE_SONAR] * tan((obst->all_objects_array[obst->all_object_index][ALL_ANGULAR_WIDTH] * (3.141516/180)) / 2); // sin((3.141516/180) * obst->object_array[obst->object_index][ANGULAR_WIDTH]) * obst->object_array[obst->object_index][DISTANCE_SONAR];
}

void update_information(obstacle* obst, robot* bot, oi_t* self) {
	bot->x = bot->x + bot->dist_traveled * cos(bot->angle * (3.141516/180)); // Update robot x coordinate
	bot->y = bot->y + bot->dist_traveled * sin(bot->angle * (3.141516/180)); // Update robot y coordinate
	
	if (obst->all_object_index > 0) { // Are there objects to keep track of? If so, update the objects distance and angle in respect to the robot
		for (int i = 0; i < obst->all_object_index; i++) { // Loop through total detected objects
			obst->all_objects_array[i][ALL_DISTANCE_SONAR] = sqrt( pow(bot->x - obst->all_objects_array[i][ALL_X], 2) + pow(bot->y - obst->all_objects_array[i][ALL_Y], 2) ); // Apply distance formula
			obst->all_objects_array[i][ALL_POSITION] = atan( (obst->all_objects_array[i][ALL_Y] - bot->y) / (obst->all_objects_array[i][ALL_X] - bot->x) ); // Apply formula to find theta
		}
	}
	
	char buffer[50];
	sprintf(buffer, "\r\nBot X: %.3lf\r\nBot Y: %.3lf\r\nBot Angle: %.3lf\r\n", bot->x, bot->y, bot->angle);
	
	send_message(buffer);
	// Do not reset the angle. Bot needs to know where it was last angled to be accurate.
	bot->dist_traveled = 0; // Reset for next run
}

/*float toRad(float deg){
	return (deg*3.14159265)/180;
}*/

/*char getX(obstacle* obst, robot* bot){
	return (char) obst->cur_dist_IR * cos(toRad((obst->start_angle_IR + obst->end_angle_IR)/2));
}*/

/*char getY(obstacle* obst, robot* bot){
	return (char) obst->cur_dist_IR * sin(toRad((obst->start_angle_IR + obst->end_angle_IR)/2));
}*/

void reset(obstacle* obst, robot* bot) {
	obst->degrees = 0.0;
	move_servo(&obst->degrees);
	initializations(obst, bot); // reinitialize
	for (int i = 0; i < 15; i++) { // reset object arrays
		for (int j = 0; j < 7; j++) {
			obst->all_objects_array[i][j] = 0;
		}
	}
}

void find_objs_IR(obstacle* obst, robot* bot) {
	if (obst->cur_dist_IR <= MAX_DETECTION_DISTANCE) { // Current IR measured distance is within detection range?
		if (obst->object_detected == 0) { // If yes, are we looking for a new object?
			obst->object_detected = 1; // We've detected something
			obst->start_angle_IR = obst->degrees; // Log start angle
			obst->start_dist_SONAR = obst->cur_dist_SONAR; // Log distance measured by sonar as start distance (IR is less consistent, but computed averages anyway as backup)
			obst->validation_level++; // Begin object validation sequence
		} else if (obst->object_detected == 1) { // Continue object validation sequence if still in detection range
			obst->validation_level++;
			obst->total_dist_IR += obst->cur_dist_IR; // Find total distance measured by IR
		}
		} else {
		if ((obst->object_detected == 1 && obst->validation_level >= SMALL_OBJECT_SIZE_MIN)) { // We've finished seeing the object. Is the object valid (at least as big as smallest object size)?
			obst->end_angle_IR = obst->degrees - 1; // Log the last measured angle
			obst->end_dist_SONAR = obst->last_dist_SONAR; // Log the last distance measured by the sonar as end distance (same reason as before)
			obst->object_detected = 0; // Reset detection variable
			obst->all_objects_array[obst->all_object_index][ALL_ANGULAR_WIDTH] = obst->end_angle_IR - obst->start_angle_IR; // Log calculated object angular size
			obst->all_objects_array[obst->all_object_index][ALL_DISTANCE_SONAR] = (obst->start_dist_SONAR + obst->end_dist_SONAR) / 2; // Log calculated object distance
			obst->all_objects_array[obst->all_object_index][ALL_DISTANCE_IR] = obst->total_dist_IR / (obst->validation_level - 1); // IR Distance = Average = Sum/N (total distance/number of distance measurements), where validation level serves as N - 1 (to account for extra sample at line 113)
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = get_linear_width(obst); // Log calculated linear width
			obst->all_objects_array[obst->all_object_index][ALL_POSITION] = (obst->start_angle_IR + (obst->all_objects_array[obst->all_object_index][ALL_ANGULAR_WIDTH] / 2)); // Log calculated object angular position
			obst->all_objects_array[obst->all_object_index][ALL_X] = bot->x + obst->all_objects_array[obst->all_object_index][ALL_DISTANCE_SONAR] * cos(obst->all_objects_array[obst->all_object_index][ALL_POSITION] * (3.141516/180)); // Assign X coordinate of object in respect to the bot
			obst->all_objects_array[obst->all_object_index][ALL_Y] = bot->y + obst->all_objects_array[obst->all_object_index][ALL_DISTANCE_SONAR] * sin(obst->all_objects_array[obst->all_object_index][ALL_POSITION] * (3.141516/180)); // Assign Y coordinate of object in respect to the bot
			obst->all_object_index++; // Move to next index
			obst->validation_level = 0; // Reset validation level
		}
		else if (obst->object_detected == 1) { // Object is an anomaly. Reset last logged variables.
			obst->validation_level = 0;
			obst->start_angle_IR = 0;
			obst->start_dist_SONAR = 0;
			obst->object_detected = 0;
			obst->total_dist_IR = 0;
		}
	}
	obst->last_dist_IR = obst->cur_dist_IR; // Remember the last measured IR distance
	obst->last_dist_SONAR = obst->cur_dist_SONAR; // Remember last measured SONAR distance
}

/*void analyze_found_objects(obstacle* obst) {
	// Break objects into distinguished categories
	for (int i = 0; i < obst->all_object_index; i++) {
		if ((obst->all_objects_array[i][ALL_LINEAR_WIDTH] >= SMALL_OBJECT_SIZE_MIN && obst->all_objects_array[i][ALL_LINEAR_WIDTH] <= SMALL_OBJECT_SIZE_MAX)) {
			obst->goal_post_array[obst->goal_post_index][GOALP_DISTANCE] = obst->all_objects_array[i][ALL_DISTANCE_SONAR]; // Log the distance between the post and the bot
			obst->goal_post_array[obst->goal_post_index++][GOALP_POSITION] = obst->all_objects_array[i][ALL_POSITION];     // Log the position between the post and the bot
		} else if (obst->all_objects_array[i][ALL_LINEAR_WIDTH] >= MEDIUM_OBJECT_SIZE_MIN && obst->all_objects_array[i][ALL_LINEAR_WIDTH] <= LARGE_OBJECT_SIZE_MAX) {
			obst->obst_array[obst->obst_index][OBST_DISTANCE] = obst->all_objects_array[i][ALL_DISTANCE_SONAR];            // Log the distance between the obstacle and the bot
			obst->obst_array[obst->obst_index++][OBST_POSITION] = obst->all_objects_array[i][ALL_POSITION];                // Log the position between the obstacle and the bot
		}
	}
	
	// Parse Relative Location of Goal Posts(s)
	for (int i = 0; i < obst->goal_post_index; i++) {
		if (obst->goal_post_array[i][GOALP_POSITION] > 0 && obst->goal_post_array[i][GOALP_POSITION] <= 30) {
			obst->goal_post_array[i][EAST] = 1;
		} else if (obst->goal_post_array[i][GOALP_POSITION] > 30 && obst->goal_post_array[i][GOALP_POSITION] <= 60) {
			obst->goal_post_array[i][NORTH_EAST] = 1;
		} else if (obst->goal_post_array[i][GOALP_POSITION] > 60 && obst->goal_post_array[i][GOALP_POSITION] < 120) {
			obst->goal_post_array[i][NORTH] = 1;
		} else if (obst->goal_post_array[i][GOALP_POSITION] > 120 && obst->goal_post_array[i][GOALP_POSITION] <= 150) {
			obst->goal_post_array[i][NORTH_WEST] = 1;
		} else if (obst->goal_post_array[i][GOALP_POSITION] > 150 && obst->goal_post_array[i][GOALP_POSITION] <= 180) {
			obst->goal_post_array[i][WEST] = 1;
		}
	}
	
	// Parse Relative Location of Obstacle(s)
	for (int i = 0; i < obst->obst_index; i++) {
		if (obst->obst_array[i][OBST_POSITION] > 0 && obst->obst_array[i][OBST_POSITION] <= 30) {
			obst->obst_array[i][EAST] = 1;
		} else if (obst->obst_array[i][OBST_POSITION] > 30 && obst->obst_array[i][OBST_POSITION] <= 60) {
			obst->obst_array[i][NORTH_EAST] = 1;
		} else if (obst->obst_array[i][OBST_POSITION] > 60 && obst->obst_array[i][OBST_POSITION] < 120) {
			obst->obst_array[i][NORTH] = 1;
		} else if (obst->obst_array[i][OBST_POSITION] > 120 && obst->obst_array[i][OBST_POSITION] <= 150) {
			obst->obst_array[i][NORTH_WEST] = 1;
		} else if (obst->obst_array[i][OBST_POSITION] > 150 && obst->obst_array[i][OBST_POSITION] <= 180) {
			obst->obst_array[i][WEST] = 1;
		}
	}
}*/
void find_smallest_obj(obstacle* obst) {
	for (int i = 0; i < obst->all_object_index; i++)
	if (obst->all_objects_array[i][ALL_LINEAR_WIDTH] < obst->smallest_obj_angular_size) {
		obst->smallest_obj_angular_size = obst->all_objects_array[i][ALL_ANGULAR_WIDTH];
		obst->smallest_obj_linear_size = obst->all_objects_array[i][ALL_LINEAR_WIDTH];
		obst->smallest_obj_dist_SONAR = obst->all_objects_array[i][ALL_DISTANCE_SONAR];
		obst->smallest_obj_dist_IR = obst->all_objects_array[i][ALL_DISTANCE_IR];
		obst->smallest_obj_position = obst->all_objects_array[i][ALL_POSITION];
	}
}

void find_closest_obj(obstacle* obst) {
	for (int i = 0; i < obst->all_object_index; i++)
	if (obst->all_objects_array[i][ALL_DISTANCE_SONAR] < obst->closest_obj_dist_SONAR || obst->all_objects_array[i][ALL_DISTANCE_IR] < obst->closest_obj_dist_IR) {
		obst->closest_obj_angular_size = obst->all_objects_array[i][ALL_ANGULAR_WIDTH];
		obst->closest_obj_linear_size = obst->all_objects_array[i][ALL_LINEAR_WIDTH];
		obst->closest_obj_dist_SONAR = obst->all_objects_array[i][ALL_DISTANCE_SONAR];
		obst->closest_obj_dist_IR = obst->all_objects_array[i][ALL_DISTANCE_IR];
		obst->closest_obj_position = obst->all_objects_array[i][ALL_POSITION];
	}
}

void print_and_process_stats(obstacle* obst) {
	if (obst->all_object_index > 0) {
		char buffer[500];
	
		/* Prepare buffer for transmission */
		sprintf(buffer, "\r\n\nObjects found: %d\r\n\nClosest Object Statistics:\r\nObject position: %.1f degrees\r\nSONAR distance (cm): %d\r\nIR distance (cm): %d\r\nAngular width: %d\r\nLinear width (cm): %d\r\n\nSmallest Object Statistics:\r\nObject position: %.1f degrees\r\nSONAR distance (cm): %d\r\nIR distance (cm): %d\r\nAngular width: %d\r\nLinear width (cm): %d\r\n", obst->all_object_index, obst->closest_obj_position, obst->closest_obj_dist_SONAR, obst->closest_obj_dist_IR, obst->closest_obj_angular_size, obst->closest_obj_linear_size, obst->smallest_obj_position, obst->smallest_obj_dist_SONAR, obst->smallest_obj_dist_IR, obst->smallest_obj_angular_size, obst->smallest_obj_linear_size);
		send_message(buffer);
	} else {
		send_message("\r\nNo objects found\r\n");
	}
}