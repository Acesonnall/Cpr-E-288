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
		
		c.user_command = USART_Receive();
		
		get_command(c, &obst, sensor_data, &bot);
		
		USART_Transmit(c.user_command);
	}
	
	return 0;
}

void move(oi_t *self, float distance_mm, robot *bot) { // Find more accurate way of moving robot
	float togo = distance_mm/0.11;                    // calculated sensor distance
	float travel = 0;				                    // distance traveled by robot
	bot->dist_traveled = distance_mm;
	
	if (distance_mm > 0) {
		oi_set_wheels(150, 150);
		
		while (travel < togo) { // Cliff sensors currently calibrated for bot 3
			if ((self->cliff_frontleft_signal > 900 || self->cliff_frontleft) || (self->cliff_frontright_signal > 670 || self->cliff_frontright)) {
				move(self, -distance_mm, bot);
				break;
			}
			
			if (self->cliff_left_signal > 920 || self->cliff_left) {
				move(self, -distance_mm, bot);
				break;
			}
			
			if (self->cliff_right_signal > 550 || self->cliff_right) {
				move(self, -distance_mm, bot);
				break;
			}
			
			if (self->bumper_left) {
				move(self, -15, bot);
				travel -= 15.0/0.115;
				wait_ms(100);
				rotate(self, -90, bot);
				wait_ms(100);
				break;
			}
			
			if (self->bumper_right) {
				move(self, -15, bot);
				travel -= 15.0/0.115;
				wait_ms(100);
				rotate(self, 90, bot);
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
		move(self, c.travel_dist, bot);
	} else if (c.user_command == 'a') {
		rotate(self, c.angle_to_turn, bot);
	} else if (c.user_command == 'd') {
		rotate(self, -c.angle_to_turn, bot);
	} else if (c.user_command == 'q') {
		sweep(obst, bot);
		print_and_process_stats(obst);
		reset(obst, bot, &c);
	} else {
		oi_load_song(c.s1_id, c.s1_num_notes, c.s1_notes, c.s1_duration);
		oi_play_song(c.s1_id);
	}
	update_information(obst, bot, self);
}

void read_cliff_sensors(oi_t *self) {
	lprintf("Cliff L: %d\nCliff Front L: %d\nCliff Front R: %d\nCliff R: %d", self->cliff_left_signal, self->cliff_frontleft_signal, self->cliff_frontright_signal, self->cliff_right_signal);
	//lprintf("Cliff L: %d\nCliff Front L: %d\nCliff Front R: %d\nCliff R: %d", self->cliff_left, self->cliff_frontleft, self->cliff_frontright, self->cliff_right);
	wait_ms(100);
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