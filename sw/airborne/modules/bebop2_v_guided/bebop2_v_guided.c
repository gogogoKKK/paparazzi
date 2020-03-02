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
 * @file "modules/bebop2_v_guided/bebop2_v_guided.c"
 * @author zsk
 * bebop2 vertical control in guided mode
 */

#include "modules/bebop2_v_guided/bebop2_v_guided.h"
#include <time.h>
#include "modules/bebop2_guided/bebop2_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
float sp_pos_z = 0.4f;
float sp_vel_z = 0.f;


void bebop2_v_guided_init(void) {

}

void bebop2_v_guided_periodic(void) {
    static double timestart = -1.;
    const float twopi = 2.*3.1415926;
    if (guidance_v_mode != GUIDANCE_V_MODE_GUIDED) {
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
    float freq = 0.2, A = 0.3;
//    if (trajectory_guided_mode == 0){
    sp_pos_z = A*sin(twopi*freq*dt)-0.8;
//    sp_vel_z = A*twopi*freq*cos(twopi*freq*dt);
//    }
    guidance_v_set_guided_z(sp_pos_z);
//    guidance_v_set_guided_vz(sp_vel_v);
}


