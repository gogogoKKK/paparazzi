/*
 * Copyright (C) zsk
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/bebop2_guided/bebop2_guided.c"
 * @author zsk
 * guided mode for bebop2
 */
#include <time.h>
#include "modules/bebop2_guided/bebop2_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"

double timestart = -1.;
enum navigation_state_t {
    LINE_X,
    LINE_Y,
    CIRCLE,
    ARBITRARY
};

const float twopi = 2.*3.1415926;
int trajectory_guided_mode = 4;
float sp_pos_x = 0., sp_pos_y = 0., sp_vel_x = 0., sp_vel_y = 0.;
void bebop2_guided_init(void) {}
void bebop2_guided_periodic(void){
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        return;
    }
    struct timeval now;
    gettimeofday(&now, NULL);
    double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
//    printf("guided %d %d %d %f\n", now.tv_sec, now.tv_usec, time_now_ms, time_now);

    if (timestart < 0.){
        timestart = time_now;
    }
    float dt = time_now - timestart;

    float freq = 0.2, A = 0.5;
    if (trajectory_guided_mode == 0){
        sp_pos_x = A*sin(twopi*freq*dt);
        sp_pos_y = 0.;
        sp_vel_x = A*twopi*freq*cos(twopi*freq*dt);
        sp_vel_y = 0.;
    }else if (trajectory_guided_mode == 1){
        sp_pos_y = A*sin(twopi*freq*dt);
        sp_pos_x = 0.;
        sp_vel_y = A*twopi*freq*cos(twopi*freq*dt);
        sp_vel_x = 0.;
    }else if (trajectory_guided_mode == 2){
        sp_pos_x = A*sin(twopi*freq*dt);
        sp_pos_y = A*cos(twopi*freq*dt);
        sp_vel_x = A*twopi*freq*cos(twopi*freq*dt);
        sp_vel_y = -A*twopi*freq*sin(twopi*freq*dt);
    }else if (trajectory_guided_mode == 3){
        sp_pos_y = 0.;
        sp_pos_x = 0.;
        sp_vel_y = 0.;
        sp_vel_x = 0.;
    }else{
        sp_pos_y = 0.;
        sp_pos_x = 0.;
        sp_vel_y = 0.;
        sp_vel_x = 0.;
    }
//    printf("guided %d %d %f dt: %f pos: %f vel: %f\n", now.tv_sec, now.tv_usec, time_now, dt, pos, vel);

    guidance_h_set_guided_pos(sp_pos_x, sp_pos_y);
    guidance_h_set_guided_vel(sp_vel_x, sp_vel_y);



}


