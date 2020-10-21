/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_outerloop_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_module_outerloop_demo.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "autopilot.h"
#include "subsystems/actuators/motor_mixing.h"
#include <time.h>
#include "subsystems/abi.h"
#include "modules/sensors/cameras/jevois.h"
#include "pthread.h"
#include <stdio.h>
// Own Variables
#define LOG_INFO(format, args...) (fprintf(fplog, "[INFO]-[%d]:" format, __LINE__, ##args))
//#define LOG_INFO(format, args...) (printf("[INFO]-[%d]:" format, __LINE__, ##args))
#ifndef IBVS_LOG_PATH
#define IBVS_LOG_PATH /data/ftp/internal_000/log
#endif
float ctrl_outerloop_kp = 2., ctrl_outerloop_kv = 1., ctrl_outerloop_kh = 2., ctrl_outerloop_kvz = 1.;
static abi_event jevois_ev;
FILE *fplog = NULL, *fpibvs_out = NULL;

const float twopi = 2.*3.1415926;
int trajectory_guided_mode = 0;
bool use_jevois_ctrl = false;
int jevois_start_status = 0;
bool jevois_send_start = false, jevois_send_stop = false;
const float mg = 6.4, Fhmax = .6, Fvmax = .6;
const float sp_yaw = -RadOfDeg(90.), sp_h = -.4;
const float maxdeg = RadOfDeg(10.);
pthread_mutex_t jevois_force_mutex, last_jevois_update_time_mutex;
double last_jevois_update_time = -1.;
struct FloatVect3 desired_force_jevois;

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;

// Settings
float comode_time = 0;
void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra);

double get_last_jevois_update_time(void);

void set_last_jevois_update_time(double update_time);

bool get_detection_status(void);

void set_detection_status(bool status);

void set_desired_force_jevois(float Fx, float Fy, float Fz);

void get_desired_force_jevois(float *Fx, float *Fy, float *Fz);

void create_log_file(){
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    char buf[256];
    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fplog = fopen(buf, "w");

//    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d_out.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
//            tm->tm_hour, tm->tm_min, tm->tm_sec);
//    fpibvs_out = fopen(buf, "w");
}

void HeadingWorldForce2Att(float Fx, float Fy, float Fz,
                           float *roll, float *pitch,
                           float *U);

void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra){
    if (type == JEVOIS_MSG_F3) {
        struct FloatVect3 normal, vartheta, F;
        const float precision = 1000.f;
        static unsigned counter = 0;
        struct timeval now;
        gettimeofday(&now, NULL);
        double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;

        normal.x = ((float)coord[3*0])/precision;
        normal.y = ((float)coord[3*0+1])/precision;
        normal.z = ((float)coord[3*0+2])/precision;

        vartheta.x = ((float)coord[3*1])/precision;
        vartheta.y = ((float)coord[3*1+1])/precision;
        vartheta.z = ((float)coord[3*1+2])/precision;
        LOG_INFO("normal: %f %f %f velocity: %f %f %f\n", normal.x, normal.y, normal.z, vartheta.x, vartheta.y, vartheta.z);

        // a controller should be implemented here
        set_desired_force_jevois(F.x, F.y, F.z);
        set_last_jevois_update_time(time_now);
    }else{
        LOG_INFO("height too low\n");
    }
}

void ctrl_module_outerloop_demo_jevois_status(bool activate){
    jevois_start_status = activate;
    if (activate){
        jevois_update_date();
        jevois_send_string("start\n");
    }else{
        jevois_send_string("stop\n");
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
    if (fplog){
        fclose(fplog);
        fplog = NULL;
    }
    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fplog = fopen(buf, "w");
}

void ctrl_module_outerloop_demo_init(void){
    AbiBindMsgJEVOIS_MSG(CAM_JEVOIS_ID, &jevois_ev, jevois_msg_event);
    create_log_file();
}
////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{

}

void guidance_h_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

void guidance_h_module_read_rc(void)
{
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

void guidance_h_module_run(bool in_flight)
{
    static unsigned int counter = 0;
    static double start_time = -1.;
    float Fx, Fy, Fz;
    struct timeval now;
    gettimeofday(&now, NULL);

    double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
    double dtime = fabs(time_now - get_last_jevois_update_time());
    if (start_time < 0){
        start_time = time_now;
    }
    double dtime2 = time_now - start_time;
    float freq = 0.1, A = 0.5;
    float pxned = stateGetPositionNed_f()->x;
    float pyned = stateGetPositionNed_f()->y;
    float pzned = stateGetPositionNed_f()->z;

    float roll_cmd = 0.;
    float pitch_cmd = 0.;
    float thrust_cmd = 0.;
    float sp_pos_x = 0., sp_pos_y = 0., sp_vel_x = 0., sp_vel_y = 0.;
    sp_pos_x = A*sin(twopi*freq*dtime2);
    sp_pos_y = 0.;
    sp_vel_x = A*twopi*freq*cos(twopi*freq*dtime2);
    sp_vel_y = 0.;
    if (!use_jevois_ctrl || dtime > 0.5 || fabs(pzned) > 1.5 || fabs(pxned) > 0.8 || fabs(pyned) > 0.8){
        float vxned = stateGetSpeedNed_f()->x;
        float vyned = stateGetSpeedNed_f()->y;
        float vzned = stateGetSpeedNed_f()->z;

//        struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();
        float yaw = stateGetNedToBodyEulers_f()->psi;
        float cpsi = cos(yaw);
        float spsi = sin(yaw);
        float err_pos_x = pxned - sp_pos_x;
        float err_pos_y = pyned - sp_pos_y;
        float err_vel_x = vxned - sp_vel_x;
        float err_vel_y = vyned - sp_vel_y;

        float pxrobot = cpsi*err_pos_x + spsi*err_pos_y;
        float pyrobot = -spsi*err_pos_x + cpsi*err_pos_y;

        float vxrobot = cpsi*err_vel_x + spsi*err_vel_y;
        float vyrobot = -spsi*err_vel_x + cpsi*err_vel_y;
//    printf("inflight: %d pos: %f %f %f vel: %f %f %f yaw: %f\n", in_flight, pxned, pyned, pzned, vxned, vyned, vzned, yaw);
        Fx = (-pxrobot*ctrl_outerloop_kp - vxrobot*ctrl_outerloop_kv);
        Fy = (-pyrobot*ctrl_outerloop_kp - vyrobot*ctrl_outerloop_kv);
//        Fz = (pzned - sp_h)*ctrl_outerloop_kh + (vzned)*ctrl_outerloop_kvz;
        Fz = (sp_h - pzned)*ctrl_outerloop_kh - (vzned)*ctrl_outerloop_kvz;
        BoundAbs(Fx, Fhmax);
        BoundAbs(Fy, Fhmax);
        BoundAbs(Fz, Fvmax);

        HeadingWorldForce2Att(Fx, Fy, Fz, &roll_cmd, &pitch_cmd, &thrust_cmd);
        if (counter++%20 == 0){
            LOG_INFO("use_jevois_ctrl: %d pos: %f %f %f pos_sp: %f %f %f F_security: %f %f %f\n",
                     use_jevois_ctrl, pxned, pyned, pzned, sp_pos_x, sp_pos_y, sp_h, Fx, Fy, Fz);
//            LOG_INFO("yaw: %f posrobo: %f %f roll_cmd2: %f pitch_cmd2: %f thrust_cmd2: %f\n",
//                     yaw*57.3, pxrobot, pyrobot, roll_cmd, pitch_cmd, thrust_cmd);
        }

    }
    else{
        get_desired_force_jevois(&Fx, &Fy, &Fz);
//        Fx += Fx_bais;
        BoundAbs(Fx, Fhmax);
        BoundAbs(Fy, Fhmax);
        BoundAbs(Fz, Fvmax);
        HeadingWorldForce2Att(Fx, Fy, Fz, &roll_cmd, &pitch_cmd, &thrust_cmd);
//        BodyForce2Att(Fx, Fy, Fz, &roll_cmd, &pitch_cmd, &thrust_cmd);
        if (counter++%21 == 0)
            LOG_INFO("roll_cmd: %f pitch_cmd: %f thrust_cmd: %f\n",
                     roll_cmd, pitch_cmd, thrust_cmd);
//        LOG_INFO("ibvs F: %f %f %f\n", Fx, Fy, Fz);
    }
    BoundAbs(roll_cmd, maxdeg);
    BoundAbs(pitch_cmd, maxdeg);
//    if (counter++ %12 == 0){
//        LOG_INFO("pos: %f %f %f dt: %f F: %f %f %f cmd: %f %f\n", pxned, pyned, pzned,
//                dtime, Fx, Fy, Fz, DegOfRad(roll_cmd), DegOfRad(pitch_cmd));
//    }
    ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll_cmd);
    ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch_cmd);
    ctrl.cmd.psi = ANGLE_BFP_OF_REAL(sp_yaw);
    stabilization_cmd[COMMAND_THRUST] = thrust_cmd*1000;
//    printf("thrust: %d\n", stabilization_cmd[COMMAND_THRUST]);
    stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
    stabilization_attitude_run(in_flight);
}

double get_last_jevois_update_time(void){
    pthread_mutex_lock (&last_jevois_update_time_mutex);
    double local_last_jevois_update_time = last_jevois_update_time;
    pthread_mutex_unlock(&last_jevois_update_time_mutex);
    return local_last_jevois_update_time;
}

void set_last_jevois_update_time(double update_time){
    pthread_mutex_lock(&last_jevois_update_time_mutex);
    last_jevois_update_time = update_time;
    pthread_mutex_unlock(&last_jevois_update_time_mutex);
}

void set_desired_force_jevois(float Fx, float Fy, float Fz){
    pthread_mutex_lock(&jevois_force_mutex);
    desired_force_jevois.x = Fx;
    desired_force_jevois.y = Fy;
    desired_force_jevois.z = Fz;
    pthread_mutex_unlock(&jevois_force_mutex);
}

void get_desired_force_jevois(float *Fx, float *Fy, float *Fz){
    pthread_mutex_lock(&jevois_force_mutex);
    *Fx = desired_force_jevois.x;
    *Fy = desired_force_jevois.y;
    *Fz = desired_force_jevois.z;
    pthread_mutex_unlock(&jevois_force_mutex);
}

void HeadingWorldForce2Att(float Fx, float Fy, float Fz,
                           float *roll, float *pitch,
                           float *U){
    float a = mg - Fz;
    *U = sqrt(a*a + Fx*Fx + Fy*Fy);
    *roll = asin(Fy/(*U));
    *pitch = asin(-Fx/((*U)*cos(*roll)));
}

