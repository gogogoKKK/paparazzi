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
int trajectory_vguided_mode = 0;
#define MAX_NZ 20
float rand_sp_pos_z[MAX_NZ], rand_sp_zpos_t[MAX_NZ];

double findModz(double a, double b)
{
    double mod;
    // Handling negative values
    if (a < 0)
        mod = -a;
    else
        mod =  a;
    if (b < 0)
        b = -b;
    // Finding mod by repeated subtraction
    while (mod >= b)
        mod = mod - b;
    // Sign of result typically depends
    // on sign of a.
    if (a < 0)
        return -mod;
    return mod;
}

void bebop2_v_guided_init(void) {
    time_t t;
    srand((unsigned) time(&t));
//    int range = 500; //-range ~ range, less than at least 32767 in unit mm
    float range = 0.5;
    for (int i = 0; i<MAX_NZ; i++){
        rand_sp_pos_z[i] = 2*range*((double)rand())/RAND_MAX-range-1.2;
//        printf("rand_sp_posz: %f\n", rand_sp_pos_z[i]);
    }
    float sp_v = 0.5;
    rand_sp_zpos_t[0] = 0.;
    for (int i = 0; i<MAX_NZ-1; i++){
        float s = fabs(rand_sp_pos_z[i+1] - rand_sp_pos_z[i]);
        rand_sp_zpos_t[i+1] = rand_sp_zpos_t[i]+s/sp_v;
    }
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
    const float freq = 0.2, A = 0.5;
    if (trajectory_vguided_mode == 0){
        sp_pos_z = A*sin(twopi*freq*dt)-0.9;
//    sp_vel_z = A*twopi*freq*cos(twopi*freq*dt);
    }else if (trajectory_vguided_mode == 1){
        dt = findModz(dt, rand_sp_zpos_t[MAX_NZ-1]);
        for (int i = 0; i<MAX_NZ-1; i++){
            if(dt >= rand_sp_zpos_t[i] && dt < rand_sp_zpos_t[i+1]){
                sp_pos_z = rand_sp_pos_z[i];
                break;
            }
            else{
                sp_pos_z = 0.;
            }
        }
//        printf("dt: %f z: %f\n", dt, sp_pos_z);
    }else{
        sp_pos_z = -0.7;
    }
    guidance_v_set_guided_z(sp_pos_z);
//    guidance_v_set_guided_vz(sp_vel_v);
}


