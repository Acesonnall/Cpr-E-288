/*
 * Final Project.c
 *
 * Created: 3/28/2016 2:00:15 PM
 * Author : Omar Taylor, Dalton Handel, Louis Hamilton, Souparni Agnihotri
 */ 

#include <avr/io.h>
#include "object_tracking.h"
#include "open_interface.h"
#include "util.h"
#include "lcd.h"
#include "main.h"
#include <math.h>

int main(void)
{
	control c;
	obstacle obst;
	robot bot;
    oi_t *sensor_data = oi_alloc();
	bot.initialized = 0; // Has to be called only once and before reset
	reset(&obst, &bot, &c);
    oi_init(sensor_data);
	
	while (1) {
		oi_update(sensor_data);
		
		//read_cliff_sensors(sensor_data);
		
		c.user_command = USART_Receive();
		
		get_command(c, &obst, sensor_data, &bot);
		
		USART_Transmit(c.user_command);
	}
	
	return 0;
}

void move(oi_t *self, float distance_mm, obstacle* obst, robot* bot, control c) { // Find more accurate way of moving robot
	float togo = distance_mm/0.11;                    // calculated sensor distance
	float travel = 0;				                    // distance traveled by robot
	bot->dist_traveled = distance_mm;
	
	if (distance_mm > 0) {
		oi_set_wheels(150, 150);
		
		while (travel < togo) { // [Bot 17]: White Tape -- CFL = 300, CFR = 300, L = 450 , R = 650; Red Tape -- CFL = >450, CFR = >800, L =  >600, R = >800
			wait_ms(5);
			if (((self->cliff_frontleft_signal > 740 && self->cliff_frontleft_signal < 850) || self->cliff_frontleft) || (self->cliff_frontright_signal > 350 && self->cliff_frontright_signal < 460) || self->cliff_frontright) {
				if (self->cliff_frontleft || self->cliff_frontright)
					log_position(obst, bot, MIDDLE, CLIFF, (distance_mm - travel)/10);
				else
					log_position(obst, bot, MIDDLE, WHITE, (distance_mm - travel)/10);
				move(self, -distance_mm, obst, bot, c);
				break;
			} else if (self->cliff_frontleft_signal > 1100 || self->cliff_frontright_signal > 640) { // Red Tape Found
				log_position(obst, bot, MIDDLE, RED, (distance_mm - travel)/10);
				oi_load_song(c.s2_id, c.s2_num_notes, c.s2_notes, c.s2_duration);
				oi_play_song(c.s2_id);
			}
			
			if ((self->cliff_left_signal > 450 && self->cliff_frontleft_signal < 560) || self->cliff_left) {
				if (self->cliff_left)
					log_position(obst, bot, LEFT, CLIFF, (distance_mm - travel)/10);
				else
					log_position(obst, bot, LEFT, WHITE, (distance_mm - travel)/10);
				move(self, -distance_mm, obst, bot, c);
				break;
			} else if (self->cliff_left_signal > 780) { // Found Red Tape
				log_position(obst, bot, LEFT, RED, (distance_mm - travel)/10);
				oi_load_song(c.s2_id, c.s2_num_notes, c.s2_notes, c.s2_duration);
				oi_play_song(c.s2_id);
			}
			
			if ((self->cliff_right_signal > 480 && self->cliff_frontright_signal < 560) || self->cliff_right) {
				if (self->cliff_right)
					log_position(obst, bot, RIGHT, CLIFF, (distance_mm - travel)/10);
				else
					log_position(obst, bot, RIGHT, WHITE, (distance_mm - travel)/10);
				move(self, -distance_mm, obst, bot, c);
				break;
			} else if (self->cliff_right_signal > 760) { // Found Red Tape
				log_position(obst, bot, RIGHT, RED, (distance_mm - travel)/10);
				oi_load_song(c.s2_id, c.s2_num_notes, c.s2_notes, c.s2_duration);
				oi_play_song(c.s2_id);
			}
			
			if (self->bumper_left && self->bumper_right) {
				oi_set_wheels(0, 0);
				log_position(obst, bot, MIDDLE, FLAT, (distance_mm - travel)/10); // divided by 10 to convert to cm
				move(self, (distance_mm - travel)/10, obst, bot, c);
				travel -= ((distance_mm - travel)/10)/0.115;
				wait_ms(100);
				break;
			} else if (self->bumper_left) {
				oi_set_wheels(0, 0);
				log_position(obst, bot, LEFT, FLAT, (distance_mm - travel)/10);
				move(self, (distance_mm - travel)/10, obst, bot, c);
				travel -= ((distance_mm - travel)/10)/0.115;
				wait_ms(100);
				break;
			} else if (self->bumper_right) {
				oi_set_wheels(0, 0);
				log_position(obst, bot, RIGHT, FLAT, (distance_mm - travel)/10);
				move(self, (distance_mm - travel)/10, obst, bot, c);
				travel -= ((distance_mm - travel)/10)/0.115;
				wait_ms(100);
				break;
			}
			oi_update(self);
			travel += self->distance;
		}
	} else if (distance_mm < 0) {
		oi_set_wheels(-150, -150);
		
		while (travel > togo) {
			oi_update(self);
			travel += self->distance;
		}
	}
	
	oi_set_wheels(0, 0);
}

void rotate(oi_t *self, float degrees, robot* bot) {
		float sensordegrees = degrees/1.1; //calibration: make number smaller to oversteer.
		float toturn = 0;
		
		bot->angle += degrees;
		
		if (bot->angle < 0) {
			bot->angle = 360 + bot->angle;
			} else if (bot->angle > 360) {
			bot->angle = bot->angle - 360;
		}
		
		if (degrees > 0){ //rotate CCW
			oi_set_wheels(100,-100);
			while (toturn < sensordegrees) {
				oi_update(self);
				toturn += self->angle;
			}
		}
		if (degrees < 0){ //rotate CW
			oi_set_wheels(-100,100);
			while (toturn > sensordegrees) {
				oi_update(self);
				toturn += self->angle;
			}
		}
		oi_set_wheels(0, 0); // stop
}

void get_command(control c, obstacle* obst, oi_t *self, robot* bot) {
	if (c.user_command == 'w') {
		move(self, c.travel_dist, obst, bot, c);
	} else if (c.user_command == 'a') {
		rotate(self, c.angle_to_turn, bot);
	} else if (c.user_command == 'd') {
		rotate(self, -c.angle_to_turn, bot);
	} else if (c.user_command == 'q') {
		sweep(obst, bot);
		//print_and_process_stats(obst);
		reset(obst, bot, &c);
	} else if (c.user_command == 'r') {
		reset_object_array(obst);
	} else if (c.user_command == 'b') {
		reinitialize_bot(bot);
	} else if (c.user_command == '1') {
		oi_load_song(c.s1_id, c.s1_num_notes, c.s1_notes, c.s1_duration);
		oi_play_song(c.s1_id);
	}
	update_information(obst, bot, self);
}

void read_cliff_sensors(oi_t *self) {
	lprintf("Cliff L: %d\nCliff Front L: %d\nCliff Front R: %d\nCliff R: %d", self->cliff_left_signal, self->cliff_frontleft_signal, self->cliff_frontright_signal, self->cliff_right_signal);
	//lprintf("Cliff L: %d\nCliff Front L: %d\nCliff Front R: %d\nCliff R: %d", self->cliff_left, self->cliff_frontleft, self->cliff_frontright, self->cliff_right);
	wait_ms(300);
}

void log_position(obstacle* obst, robot* bot, char bumper_cliff, char object, signed char dist) {
	dist *= -1; // When backing up, we need to pass in the distance we backed up.
	if (bumper_cliff == LEFT) {
		if (object == CLIFF) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = CLIFF;
		} else if (object == WHITE) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = WHITE;
		} else if(object == RED) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = RED;
		} else {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = FLAT;
		}
		
	} else if (bumper_cliff == MIDDLE) {
		if (object == CLIFF) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = CLIFF;
		} else if (object == WHITE) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = WHITE;
		} else if (object == RED) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = RED;
		} else {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = FLAT;
		}
	} else if (bumper_cliff == RIGHT) {
		if (object == CLIFF) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = CLIFF;
		} else if(object == WHITE) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = WHITE;
		} else if(object == RED) {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = RED;
		} else {
			log_position_helper(obst, bot, dist);
			obst->all_objects_array[obst->all_object_index][ALL_LINEAR_WIDTH] = FLAT;
		}
	}
	
	obst->all_object_index++;
	bot->dist_traveled = dist;
}

void log_position_helper(obstacle* obst, robot* bot, signed char dist) {
	obst->all_objects_array[obst->all_object_index][ALL_DISTANCE_SONAR] = dist; // Obvious
	obst->all_objects_array[obst->all_object_index][ALL_X] = (bot->x + dist * (cos((bot->angle + 45) * (3.141516/180)))) + (12 + dist) * (cos(bot->angle + 45) * (3.141516/180)); // distance value is 12 (+ 15 to account for when the bot backs up)
	obst->all_objects_array[obst->all_object_index][ALL_Y] = (bot->y + dist * (sin((bot->angle + 45) * (3.141516/180)))) + (12 + dist) * (sin(bot->angle + 45) * (3.141516/180)); // distance value is 12 (+ 15 to account for when the bot backs up)
}

/*void rendezvous(obstacle* obst, robot* bot) {
	char can_complete_rendevous = 0; // 0: No, 1: Yes
	if (obst->all_object_index > 0) {
		Fill Goal Post Array
		for (int i = 0; i < obst->all_object_index; i++) {
			if (obst->all_objects_array[i][ALL_LINEAR_WIDTH] > SMALL_OBJECT_SIZE_MAX)
				continue;
			obst->goal_post_array[obst->goal_post_index][0] = obst->all_objects_array[i][ALL_X];
			obst->goal_post_array[obst->goal_post_index][1] = obst->all_objects_array[i][ALL_Y];	
			obst->goal_post_index++;
		}
		
		for (int i = 0; i < obst->all_object_index; i++) {
			Is There An Object In The Way?
			if (obst->all_object_index[])
		}
		
	}
	
	if (!can_complete_rendevous) {
		Reset Map
		for (int i = 0; i < obst->goal_post_index; i++) {
			obst->goal_post_array[i][0] = 0;
			obst->goal_post_array[i][1] = 0;
		}
		obst->goal_post_index = 0;
		send_message("\r\nCannot find rendezvous point!\r\n");
	}
}*/