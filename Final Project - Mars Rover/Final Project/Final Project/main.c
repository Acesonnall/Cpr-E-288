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
#include "ai.h"
#include <math.h>

// Prototypes
void move(oi_t *self, float distance_mm, robot *bot);
void rotate(oi_t *self, float degrees, robot *bot);

int main(void)
{
	obstacle obst;
	robot bot;
    oi_t *sensor_data = oi_alloc();
	reset(&obst, &bot);
    oi_init(sensor_data);
	
	char dist_to_travel = 15; // cm
	char angle_to_turn = 45;  // degrees
	
	unsigned char current;
	unsigned char num_notes = 3;
	unsigned char notes[] = {36, 36, 36};
	unsigned char duration[] = {15, 15, 15};
	
	while (1) {
		oi_update(sensor_data);
		/*lprintf("Cliff L: %d\nCliff Front L: %d\nCliff Front R: %d\nCliff R: %d", sensor_data->cliff_left_signal, sensor_data->cliff_frontleft_signal, sensor_data->cliff_frontright_signal, sensor_data->cliff_right_signal);
		lprintf("Cliff L: %d\nCliff Front L: %d\nCliff Front R: %d\nCliff R: %d", sensor_data->cliff_left, sensor_data->cliff_frontleft, sensor_data->cliff_frontright, sensor_data->cliff_right);
		wait_ms(100);*/
		
		current = USART_Receive();
		
		if (current == 'w') {
			move(sensor_data, dist_to_travel, &bot);
			update_information(&obst, &bot, sensor_data);
		} else if (current == 'a') {
			rotate(sensor_data, angle_to_turn, &bot);
		} else if (current == 'd') {
			rotate(sensor_data, -angle_to_turn, &bot);
		} else if (current == 'q') {
			sweep(&obst, &bot);
			print_and_process_stats(&obst);
			reset(&obst, &bot);
		} else {
			oi_load_song(2, num_notes, notes, duration);
			oi_play_song(2);
		}
		
		USART_Transmit(current);
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
		float sensordegrees = degrees/1.13; //calibration: make number smaller to oversteer.
		float toturn = 0;
		bot->angle += fmod(degrees, 360);
		
		
		if (degrees > 0){ //rotate CCW
			oi_set_wheels(150,-150);
			while (toturn < sensordegrees) {
				oi_update(self);
				toturn += self->angle;
			}
		}
		if (degrees < 0){ //rotate CW
			oi_set_wheels(-150,150);
			while (toturn > sensordegrees) {
				oi_update(self);
				toturn += self->angle;
			}
		}
		oi_set_wheels(0, 0); // stop
}