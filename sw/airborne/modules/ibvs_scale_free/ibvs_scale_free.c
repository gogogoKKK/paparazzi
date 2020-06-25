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
 * @file "modules/ibvs_scale_free/ibvs_scale_free.c"
 * @author zsk
 * 
 */

#include "modules/ibvs_scale_free/ibvs_scale_free.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "autopilot.h"
#include "subsystems/actuators/motor_mixing.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include "pthread.h"
//#include <fcntl.h>   /* File Control Definitions           */
//#include <termios.h> /* POSIX Terminal Control Definitions */
//#include <unistd.h>  /* UNIX Standard Definitions 	   */
//#include <errno.h>   /* ERROR Number Definitions           */
//#include "sw/airborne/math/pprz_algebra_float.h"
#define LOG_INFO(format, args...) (printf("[INFO]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))
//#define LOG_INFOF(format, args...) (fprintf(fplog, "[INFO]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))

//#define Msg_Debug(format, args...) (printf("[DEBUG]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))
//
//#define Msg_Warn(format, args...) (printf("[WARN]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))
//
//#define Msg_Error(format, args...) (printf("[ERROR]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))

const float jevois_fx = 406.7542857f,
        jevois_fy = 408.1222019f,
        jevois_cx = 321.65766064f,
        jevois_cy = 245.95284275f;

struct ctrl_module_demo_struct {
// RC Inputs
    struct Int32Eulers rc_sp;

// Output command
    struct Int32Eulers cmd;

} ctrl;

static abi_event jevois_ev;
//#define LOG_PATH /data
float ctrl_outerloop_kp = 1., ctrl_outerloop_kv = .3, ctrl_outerloop_kh = 20000., ctrl_outerloop_kvz = 12000.;
FILE *fplog = NULL;
struct FloatVect3 desiredForce;
float ibvs_h_k0 = 1., ibvs_h_k1 = 1., ibvs_v_k0 = 1., ibvs_v_k1 = 1.;
const float hground = -0.1;
pthread_mutex_t mutex;
void mkdir(char* dir);

static void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                             uint8_t nb, int16_t * coord, uint16_t * dim,
                             struct FloatQuat quat, char * extra);

void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra){



//    int timeint = time_now*1e3;
//    printf("guided %d %d %d %f\n", now.tv_sec, now.tv_usec, time_now_ms, time_now);
//    printf("[receive] sender_id: %f %u type %u nb: %u\n", time_now, sender_id, type, nb);
    if (type == 23){
        struct timeval now;
        gettimeofday(&now, NULL);
        double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
        struct FloatVect3 imgcoord[4], q, b, vartheta, nu, normal, velb, velned, desiredb, delta0, delta1;

        FLOAT_VECT3_ZERO(q);
        FLOAT_VECT3_ZERO(desiredb);
        desiredb.z = 5.;
        q.x = 0.;
        q.y = 0.;
        q.z = 0.;
        float c = 0.;
        struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();

        normal.x = Rmat->m[2];
        normal.y = Rmat->m[5];
        normal.z = Rmat->m[8];
//        LOG_INFO("n")
        for (int i = 0; i<nb; i++){
            imgcoord[i].y = (coord[2*i]-jevois_cx)/jevois_fx;
            imgcoord[i].x = -(coord[2*i+1]-jevois_cy)/jevois_fy;
            imgcoord[i].z = 1.;
            float_vect3_normalize(imgcoord+i);
            float ctheta = imgcoord[i].x*normal.x + imgcoord[i].y*normal.y + imgcoord[i].z*normal.z;
            imgcoord[i].x /= ctheta;
            imgcoord[i].y /= ctheta;
            imgcoord[i].z /= ctheta;
            c += float_vect3_norm2(imgcoord+i);

            q.x += imgcoord[i].x;
            q.y += imgcoord[i].y;
            q.z += imgcoord[i].z;
//            printf(" coord %d, %d\n", coord[2*i], coord[2*i+1]);
        }
        q.x /= nb;
        q.y /= nb;
        q.z /= nb;

        c /= nb;
        c -= float_vect3_norm2(&q);
        float beta = sqrt(c);
        b.x = q.x/beta;
        b.y = q.y/beta;
        b.z = q.z/beta;

        velned.x = stateGetSpeedNed_f()->x;
        velned.y = stateGetSpeedNed_f()->y;
        velned.z = stateGetSpeedNed_f()->z;

        float_rmat_vmult(&velb, Rmat, &velned);
//        float pxned = stateGetPositionNed_f()->x;
//        float pyned = stateGetPositionNed_f()->y;
        float pzned = stateGetPositionNed_f()->z;
        float height = hground - pzned;
        if (fabs(height) < 1e-3){
            LOG_INFO("height too low\n");
        }else{
            vartheta.x = velb.x/height;
            vartheta.y = velb.y/height;
            vartheta.z = velb.z/height;
            LOG_INFO("velned: %f %f %f velb: %f %f %f\n", velned.x, velned.y, velned.z, velb.x, velb.y, velb.z);
            nu.x = vartheta.x/beta;
            nu.y = vartheta.y/beta;
            nu.z = vartheta.z/beta;
//            desiredForce.x = ibvs_h_k1*ibvs_h_k0*(b.x-desiredb.x);
//            desiredForce.y = ibvs_h_k1*ibvs_h_k0*(b.x-desiredb.x);
//            desiredForce.z = ibvs_v_k1*ibvs_v_k0*(b.x-desiredb.x);
            desiredForce.x = ibvs_h_k1*ibvs_h_k0*(b.x-desiredb.x) - ibvs_h_k1*nu.x;
            desiredForce.y = ibvs_h_k1*ibvs_h_k0*(b.x-desiredb.x) - ibvs_h_k1*nu.y;
            desiredForce.z = ibvs_v_k1*ibvs_v_k0*(b.x-desiredb.x) - ibvs_v_k1*nu.z;
            LOG_INFO("n: %f %f %f nu: %f %f %f beta: %f b: %f %f %f F: %f %f %f\n",
                    normal.x, normal.y, normal.z, nu.x, nu.y, nu.z,
                    beta, b.x, b.y, b.z, desiredForce.x, desiredForce.y, desiredForce.z);
        }
    }else if(type == 10){
        LOG_INFO("no detections nb: %d coord: %d\n", nb, coord[0]);
    }else if(type == 20){
        LOG_INFO("no detections\n");
    }

//  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
//  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);
//  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(yaw);
//  stabilization_cmd[COMMAND_THRUST] = 1000;
//  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
//  stabilization_attitude_run(in_flight);
}
void ibvs_scale_free_init(void){
//    AbiBindMsgJEVOIS_MSG(CAM_JEVOIS_ID, &jevois_ev, jevois_msg_event);
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    char log_dir[256];
    sprintf(log_dir, "/data/ibvs/");
    mkdir(log_dir);
    char buf[256];
    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d.txt", log_dir, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fplog = fopen(buf, "w");

}
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
//
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
    // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
    // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

    float vxned = stateGetSpeedNed_f()->x;
    float vyned = stateGetSpeedNed_f()->y;
    float vzned = stateGetSpeedNed_f()->z;

    float pxned = stateGetPositionNed_f()->x;
    float pyned = stateGetPositionNed_f()->y;
    float pzned = stateGetPositionNed_f()->z;

    struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();
    float yaw = stateGetNedToBodyEulers_f()->psi;
    float cpsi = cos(yaw);
    float spsi = sin(yaw);

    float pxrobot = cpsi*pxned+spsi*pyned;
    float pyrobot = -spsi*pxned+cpsi*pyned;

    float vxrobot = cpsi*vxned+spsi*vyned;
    float vyrobot = -spsi*vxned+cpsi*vyned;

    float sp_yaw = -RadOfDeg(90.), sp_h = -0.8;

//    printf("inflight: %d pos: %f %f %f vel: %f %f %f yaw: %f\n", in_flight, pxned, pyned, pzned, vxned, vyned, vzned, yaw);

    float maxdeg = RadOfDeg(15.);
    const float mg = 6., Fhmax = 1., Fvmax = 7.;
    float Fx = (-pxrobot*ctrl_outerloop_kp - vxrobot*ctrl_outerloop_kv);
    float Fy = (-pyrobot*ctrl_outerloop_kp - vyrobot*ctrl_outerloop_kv);
    float Fz = (pzned - sp_h)*ctrl_outerloop_kp+(vzned)*ctrl_outerloop_kv;
    BoundAbs(Fz, mg);
    BoundAbs(Fx, Fhmax);
    BoundAbs(Fy, Fhmax);
    Fz += mg;
    Bound(Fz, 0, Fvmax);
//    BoundAbs(Fz, Fvmax);
    float normF = sqrt(Fx*Fx+Fy*Fy+Fz*Fz);
    float roll_cmd = asin(Fy/normF);
    float pitch_cmd = atan2(-Fx, Fz);
    BoundAbs(roll_cmd, maxdeg);
    BoundAbs(pitch_cmd, maxdeg);
    printf("probot: %f %f F: %f %f %f cmd: %f %f\n", pxrobot, pyrobot, Fx, Fy, Fz, DegOfRad(roll_cmd), DegOfRad(pitch_cmd));
    ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll_cmd);
    ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch_cmd);
    ctrl.cmd.psi = ANGLE_BFP_OF_REAL(sp_yaw);

    stabilization_cmd[COMMAND_THRUST] = Fz*1000;
//    printf("thrust: %d\n", stabilization_cmd[COMMAND_THRUST]);
    stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
    stabilization_attitude_run(in_flight);
}

void mkdir(char* dir){
    if (access(dir, F_OK)) {
        char save_dir_cmd[256];
        sprintf(save_dir_cmd, "mkdir -p %s", dir);
        if (system(save_dir_cmd) != 0) {
            printf("[save_imu] Could not create images directory %s.\n", dir);
            return;
        }
    }
}