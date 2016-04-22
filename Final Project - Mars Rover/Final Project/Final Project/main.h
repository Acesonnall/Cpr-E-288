/*
 * main.h
 *
 * Created: 4/16/2016 12:33:35 PM
 *  Author: Omar Taylor
 */

void move(oi_t *self, float distance_mm, obstacle* obst, robot* bot, control c);

void rotate(oi_t *self, float degrees, robot *bot);

void get_command(control c, obstacle* obst, oi_t *self, robot* bot);

void read_cliff_sensors(oi_t *self);

void log_position(obstacle* obst, robot* bot, char bumper_cliff, char object, signed char dist);

void log_position_helper(obstacle* obst, robot* bot, signed char dist);