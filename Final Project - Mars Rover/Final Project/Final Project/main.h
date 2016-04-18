/*
 * main.h
 *
 * Created: 4/16/2016 12:33:35 PM
 *  Author: Omar Taylor
 */

void move(oi_t *self, float distance_mm, robot *bot);

void rotate(oi_t *self, float degrees, robot *bot);

void get_command(control c, obstacle* obst, oi_t *self, robot* bot);

void read_cliff_sensors(oi_t *self);