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
#include <stdio.h>
#include <stdlib.h>
#include "modules/sensors/cameras/jevois.h"
double timestart = -1.;
enum navigation_state_t {
    LINE_X,
    LINE_Y,
    CIRCLE,
    ARBITRARY
};

const float twopi = 2.*3.1415926;
int trajectory_guided_mode = 0;
float sp_pos_x = 0., sp_pos_y = 0., sp_vel_x = 0., sp_vel_y = 0.;

int jevois_start_status = 0;
bool jevois_send_start = false, jevois_send_stop = false;
#define MAX_N 50
float rand_sp_pos_x[MAX_N], rand_sp_pos_y[MAX_N], rand_sp_pos_t[MAX_N];
void bebop2_guided_init(void) {
    time_t t;
    srand((unsigned) time(&t));
    float range = 0.7;
    for (int i = 0; i<MAX_N; i++){
        rand_sp_pos_x[i] = 2*range*((double)rand())/RAND_MAX-range;
        rand_sp_pos_y[i] = 2*range*((double)rand())/RAND_MAX-range;
//        printf("rand_sp_pos: %f %f\n", rand_sp_pos_x[i], rand_sp_pos_y[i]);
    }
    float sp_v = 1.5;
    rand_sp_pos_t[0] = 0.;
    for (int i = 0; i<MAX_N-1; i++){
        float dx = rand_sp_pos_x[i+1] - rand_sp_pos_x[i];
        float dy = rand_sp_pos_y[i+1] - rand_sp_pos_y[i];
        float s = sqrt(dx*dx+dy*dy);
        rand_sp_pos_t[i+1] = rand_sp_pos_t[i]+s/sp_v;
    }
}

void jevois_update_date(){
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    char buf[256];
    sprintf(buf, "date %02d%02d%02d%02d00\n", tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min);
//    LOG_INFO("%s", buf);
    jevois_send_string(buf);
//    if (fplog){
//        fclose(fplog);
//        fplog = NULL;
//    }
//    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
//            tm->tm_hour, tm->tm_min, tm->tm_sec);
//    fplog = fopen(buf, "w");
}


double findMod(double a, double b)
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

void bebop2_guided_periodic(void){
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        return;
    }
    if (jevois_start_status == 0){
        if (jevois_send_stop){
            jevois_send_string("stop\n");
            jevois_send_stop = false;
        }
        jevois_send_start = true;
    }else{
        if (jevois_send_start){
            jevois_update_date();
            jevois_send_string("start\n");
            jevois_send_start = false;
        }
        jevois_send_stop = true;
    }

    struct timeval now;
    gettimeofday(&now, NULL);
    double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
//    printf("guided %d %d %d %f\n", now.tv_sec, now.tv_usec, time_now_ms, time_now);

    if (timestart < 0.){
        timestart = time_now;
    }
    float dt = time_now - timestart;

    float freq = 0.1, A = 0.5;
    if (trajectory_guided_mode == 0){
        sp_pos_x = A*sin(twopi*freq*dt);
        sp_pos_y = 0.4;
        sp_vel_x = A*twopi*freq*cos(twopi*freq*dt);
        sp_vel_y = 0.;
    }else if (trajectory_guided_mode == 1){
        sp_pos_y = A*sin(twopi*freq*dt);
        sp_pos_x = 0.;
        sp_vel_y = A*twopi*freq*cos(twopi*freq*dt);
        sp_vel_x = 0.;
    }else if (trajectory_guided_mode == 2){
        sp_pos_x = A*sin(twopi*freq*dt)+0.;
        sp_pos_y = A*cos(twopi*freq*dt)+0.;
        sp_vel_x = A*twopi*freq*cos(twopi*freq*dt);
        sp_vel_y = -A*twopi*freq*sin(twopi*freq*dt);
    }else if (trajectory_guided_mode == 3){
        dt = findMod(dt, rand_sp_pos_t[MAX_N-1]);
        for (int i = 0; i<MAX_N-1; i++){
            if(dt >= rand_sp_pos_t[i] && dt < rand_sp_pos_t[i+1]){
                sp_pos_x = rand_sp_pos_x[i];
                sp_pos_y = rand_sp_pos_y[i];
                break;
            }
            else{
                sp_pos_x = 0.;
                sp_pos_y = 0.;
            }
        }
//        printf("dt: %f sp_pos: %f %f\n", dt, sp_pos_x, sp_pos_y);
        sp_vel_y = 0.;
        sp_vel_x = 0.;
    }else{
        sp_pos_y = 0.5;
        sp_pos_x = 0.;
        sp_vel_y = 0.;
        sp_vel_x = 0.;
    }
//    printf("guided %d %d %f dt: %f pos: %f vel: %f\n", now.tv_sec, now.tv_usec, time_now, dt, pos, vel);

    guidance_h_set_guided_pos(sp_pos_x, sp_pos_y);
//    guidance_h_set_guided_vel(sp_vel_x, sp_vel_y);



}


