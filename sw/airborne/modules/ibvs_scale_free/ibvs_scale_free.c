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
//#include "sw/airborne/modules/sensors/cameras/jevois.h"
#include "modules/sensors/cameras/jevois.h"
#include "autopilot.h"
#include "subsystems/actuators/motor_mixing.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include "pthread.h"

#ifndef IBVS_LOG_PATH
#define IBVS_LOG_PATH /data/video/images
#endif
//#include "sw/airborne/math/pprz_algebra_float.h"
//#define LOG_INFO(format, args...) (printf("[INFO]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))
//#define LOG_INFO(format, args...) (fprintf(fplog, "[INFO]-[%s]-[%s]-[%d]:" format, __FILE__, __FUNCTION__ , __LINE__, ##args))

//#define LOG_INFO(format, args...) (printf("[INFO]-[%d]:" format, __LINE__, ##args))
#define LOG_INFO(format, args...) (fprintf(fplog, "[INFO]-[%d]:" format, __LINE__, ##args))
const float jevois_fx = 398.71295455f,
        jevois_fy = 398.35889573f,
        jevois_cx = 316.9367208f,
        jevois_cy = 252.27558958f;

struct ctrl_module_demo_struct {
// RC Inputs
    struct Int32Eulers rc_sp;

// Output command
    struct Int32Eulers cmd;

} ctrl;

static abi_event jevois_ev;
//#define LOG_PATH /data
float ctrl_outerloop_kp = 2., ctrl_outerloop_kv = 1., ctrl_outerloop_kh = 2., ctrl_outerloop_kvz = 1.;
FILE *fplog = NULL, *fpibvs_out = NULL;
struct FloatVect3 desired_force_ibvs;
float ibvs_hx_k0 = .3, ibvs_hx_k1 = 0.2, ibvs_hy_k0 = 0.2, ibvs_hy_k1 = 0.2, ibvs_v_k0 = 0.2, ibvs_v_k1 = .2;
//float ibvs_hx_k0 = .2, ibvs_hx_k1 = 0.2, ibvs_hy_k0 = .2, ibvs_hy_k1 = 0.2, ibvs_v_k0 = 0.2, ibvs_v_k1 = .2;
int jevois_start_status = 0;
const float hground = -0.1;
const float mg = 6.07, Fhmax = 2., Fvmax = 9.;
const float sp_yaw = -RadOfDeg(90.), sp_h = -1.2;
const float maxdeg = RadOfDeg(10.), pitch_cmd_bias = 0.0, roll_cmd_bias = -0.0;
const float sp_pos_x = -0.3, sp_pos_y = 0.;

bool jevois_send_start = false, jevois_send_stop = false, jevois_send_date = true;

pthread_mutex_t detection_status_mutex, ibvs_force_mutex, last_ibvs_update_time_mutex;
bool use_ibvs = false;
bool detection_status = false;
void mkdir(char* dir);
double last_ibvs_update_time = 0.;

void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra);

void compute_desiredb();

bool checkftdi();

double get_last_ibvs_update_time(void);

void set_last_ibvs_update_time(double update_time);

bool get_detection_status(void);

void set_detection_status(bool status);

void set_desired_force_ibvs(float Fx, float Fy, float Fz);

void get_desired_force_ibvs(float *Fx, float *Fy, float *Fz);

void HeadingWorldForce2Att(float Fx, float Fy, float Fz,
                           float *roll, float *pitch,
                           float *U);

void BodyForce2Att(float Fx, float Fy, float Fz,
                   float *roll, float *pitch,
                   float *U);

void ibvs_scale_free_jevois_status(bool activate){
    jevois_start_status = activate;
    if (activate){
        jevois_update_date();
        jevois_send_string("start\n");
    }else{
        jevois_send_string("stop\n");
    }
}

void create_log_file(){
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    char buf[256];
    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fplog = fopen(buf, "w");

    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d_out.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fpibvs_out = fopen(buf, "w");
}

void create_ibvs_out_file(){
    if (fpibvs_out){
        fclose(fpibvs_out);
        fpibvs_out = NULL;
    }
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    char buf[256];
    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d_out.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fpibvs_out = fopen(buf, "w");
}


void ibvs_scale_free_init(void){
    AbiBindMsgJEVOIS_MSG(CAM_JEVOIS_ID, &jevois_ev, jevois_msg_event);
    create_log_file();
}

void jevois_update_date(){
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    char buf[256];
    sprintf(buf, "date %02d%02d%02d%02d00\n", tm->tm_mon + 1, tm->tm_mday, tm->tm_hour-1, tm->tm_min);
    LOG_INFO("%s", buf);
    jevois_send_string(buf);

    if (fplog){
        fclose(fplog);
        fplog = NULL;
    }
    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fplog = fopen(buf, "w");

    if (fpibvs_out){
        fclose(fpibvs_out);
        fpibvs_out = NULL;
    }

    sprintf(buf, "%s/%04d_%02d_%02d_%02d_%02d_%02d_out.txt", STRINGIFY(IBVS_LOG_PATH), tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    fpibvs_out = fopen(buf, "w");
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
//    struct FloatVect3 desired_force;
    static unsigned int counter = 0;
    float Fx, Fy, Fz;
    struct timeval now;
    gettimeofday(&now, NULL);

    double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
    double dtime = fabs(time_now - get_last_ibvs_update_time());
    float pxned = stateGetPositionNed_f()->x;
    float pyned = stateGetPositionNed_f()->y;
    float pzned = stateGetPositionNed_f()->z;

    float roll_cmd = 0.;
    float pitch_cmd = 0.;
    float thrust_cmd = 0.;
    if (!use_ibvs || !get_detection_status() || dtime > 0.5 || fabs(pzned) > 1.5 || fabs(pxned) > 0.5 || fabs(pyned) > 0.5){
        float vxned = stateGetSpeedNed_f()->x;
        float vyned = stateGetSpeedNed_f()->y;
        float vzned = stateGetSpeedNed_f()->z;

//        struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();
        float yaw = stateGetNedToBodyEulers_f()->psi;
        float cpsi = cos(yaw);
        float spsi = sin(yaw);
        float err_pos_x = pxned - sp_pos_x;
        float err_pos_y = pyned - sp_pos_y;

        float pxrobot = cpsi*err_pos_x + spsi*err_pos_y;
        float pyrobot = -spsi*err_pos_x + cpsi*err_pos_y;

        float vxrobot = cpsi*vxned + spsi*vyned;
        float vyrobot = -spsi*vxned + cpsi*vyned;
//    printf("inflight: %d pos: %f %f %f vel: %f %f %f yaw: %f\n", in_flight, pxned, pyned, pzned, vxned, vyned, vzned, yaw);
        Fx = (-pxrobot*ctrl_outerloop_kp - vxrobot*ctrl_outerloop_kv);
        Fy = (-pyrobot*ctrl_outerloop_kp - vyrobot*ctrl_outerloop_kv);
//        Fz = (pzned - sp_h)*ctrl_outerloop_kh + (vzned)*ctrl_outerloop_kvz;
        Fz = (sp_h - pzned)*ctrl_outerloop_kh - (vzned)*ctrl_outerloop_kvz;
        BoundAbs(Fx, Fhmax);
        BoundAbs(Fy, Fhmax);
        BoundAbs(Fz, mg);

        HeadingWorldForce2Att(Fx, Fy, Fz, &roll_cmd, &pitch_cmd, &thrust_cmd);
//        roll_cmd += roll_cmd_bias;
//        pitch_cmd += pitch_cmd_bias;
        if (counter++%20 == 0){
            LOG_INFO("use_ibvs: %d detection_status: %d pos: %f %f %f pos_sp: %f %f %f F_security: %f %f %f\n",
                     use_ibvs, get_detection_status(), pxned, pyned, pzned, sp_pos_x, sp_pos_y, sp_h, Fx, Fy, Fz);
            LOG_INFO("yaw: %f posrobo: %f %f roll_cmd2: %f pitch_cmd2: %f thrust_cmd2: %f\n",
                     yaw*57.3, pxrobot, pyrobot, roll_cmd, pitch_cmd, thrust_cmd);
        }

    }
    else{
        get_desired_force_ibvs(&Fx, &Fy, &Fz);
        BoundAbs(Fx, Fhmax);
        BoundAbs(Fy, Fhmax);
        BoundAbs(Fz, mg);
        BodyForce2Att(Fx, Fy, Fz, &roll_cmd, &pitch_cmd, &thrust_cmd);
//        pitch_cmd += pitch_cmd_bias;
//        roll_cmd += roll_cmd_bias;
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

bool compute_gt(struct FloatVect3 *n, struct FloatVect3 *vartheta){
    struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();

    n->x = Rmat->m[2];
    n->y = Rmat->m[5];
    n->z = Rmat->m[8];

    struct FloatVect3 velb, velned;
    velned.x = stateGetSpeedNed_f()->x;
    velned.y = stateGetSpeedNed_f()->y;
    velned.z = stateGetSpeedNed_f()->z;
    float_rmat_vmult(&velb, Rmat, &velned);
    float pzned = stateGetPositionNed_f()->z;
    float height = hground - pzned;
    vartheta->x = velb.x/height;
    vartheta->y = velb.y/height;
    vartheta->z = velb.z/height;
//    LOG_INFO("velned: %f %f %f velb: %f %f %f\n", velned.x, velned.y, velned.z, velb.x, velb.y, velb.z);
    return fabs(height) > 0.01;
}

void compute_ibvs_F(struct FloatVect3 *imgcoord, struct FloatVect3 *normal, struct FloatVect3 *vartheta,
        struct FloatVect3 *F, struct FloatVect3 *desiredb, struct FloatVect3 *b){
    struct FloatVect3 q, nu;
    FLOAT_VECT3_ZERO(q);
    q.x = 0.;
    q.y = 0.;
    q.z = 0.;
    float c = 0.;
    const int nb = 4;
    for (int i = 0; i < nb; i++){
        float_vect3_normalize(imgcoord+i);
        float ctheta = imgcoord[i].x*normal->x + imgcoord[i].y*normal->y + imgcoord[i].z*normal->z;
        imgcoord[i].x /= ctheta;
        imgcoord[i].y /= ctheta;
        imgcoord[i].z /= ctheta;
        c += float_vect3_norm2(imgcoord+i);

        q.x += imgcoord[i].x;
        q.y += imgcoord[i].y;
        q.z += imgcoord[i].z;
//            LOG_INFO(" coord %d, %d\n", coord[2*i], coord[2*i+1]);
    }
    q.x /= nb;
    q.y /= nb;
    q.z /= nb;

    c /= nb;
    c -= float_vect3_norm2(&q);
    float beta = sqrt(c);
    b->x = q.x/beta;
    b->y = q.y/beta;
    b->z = q.z/beta;

    nu.x = vartheta->x/beta;
    nu.y = vartheta->y/beta;
    nu.z = vartheta->z/beta;
    F->x = ibvs_hx_k0*(b->x - desiredb->x) - ibvs_hx_k1*nu.x;
    F->y = ibvs_hy_k0*(b->y - desiredb->y) - ibvs_hy_k1*nu.y;
    F->z = ibvs_v_k0*(b->z - desiredb->z) - ibvs_v_k1*nu.z;
//    F->z = -F->z;
    LOG_INFO("beta: %f nu: %f %f %f b: %f %f %f desiredb_ibvs: %f %f %f F_ibvs: %f %f %f\n", beta,
             nu.x, nu.y, nu.z,
             b->x, b->y, b->z,
             desiredb->x, desiredb->y, desiredb->z,
             F->x, F->y, F->z);
//    static unsigned int counter = 0;
//    if (counter++%5 == 0)
    {
//        LOG_INFO("n: %f %f %f nu: %f %f %f beta: %f b: %f %f %f F: %f %f %f\n",
//                 normal->x, normal->y, normal->z, nu.x, nu.y, nu.z,
//                 beta, b.x, b.y, b.z, F->x, F->y, F->z);

//        LOG_INFO("nu: %f %f %f beta: %f b: %f %f %f F: %f %f %f\n",
//                 nu.x, nu.y, nu.z,
//                 beta, b.x, b.y, b.z,
//                 F->x, F->y, F->z);
    }
}

void compute_desiredb(struct FloatVect3 *desiredbRotate){
    struct FloatRMat R0, Rc0;
    R0.m[0] = 0.;
    R0.m[1] = -1.;
    R0.m[2] = 0.;
    R0.m[3] = 1.;
    R0.m[4] = 0.;
    R0.m[5] = 0.;
    R0.m[6] = 0.;
    R0.m[7] = 0.;
    R0.m[8] = 1.;
    struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();
    float_rmat_comp_inv(&Rc0, Rmat, &R0);
    struct FloatVect3 desiredb;
    desiredb.x = 0.;
    desiredb.y = 0.;
    desiredb.z = 7.;
    float_rmat_vmult(desiredbRotate, &Rc0, &desiredb);
//    LOG_INFO("Rmat: %f %f %f %f %f %f %f %f %f\n",
//             Rc0.m[0], Rc0.m[1], Rc0.m[2],
//             Rc0.m[3], Rc0.m[4], Rc0.m[5],
//             Rc0.m[6], Rc0.m[7], Rc0.m[8]);
//    LOG_INFO("Rmat: %f %f %f %f %f %f %f %f %f\n",
//             Rmat->m[0], Rmat->m[1], Rmat->m[2],
//             Rmat->m[3], Rmat->m[4], Rmat->m[5],
//             Rmat->m[6], Rmat->m[7], Rmat->m[8]);
}

void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra){
//    int timeint = time_now*1e3;
//    printf("guided %d %d %d %f\n", now.tv_sec, now.tv_usec, time_now_ms, time_now);
//    printf("[receive] sender_id: %f %u type %u nb: %u\n", time_now, sender_id, type, nb);
    static int nodetection_count = 0;
    if (type == JEVOIS_MSG_F2){
        struct timeval now;
        gettimeofday(&now, NULL);
        double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
        struct FloatVect3 normal, vartheta;
//        bool gtsuccess = compute_gt(&normal, &vartheta);
//        if (gtsuccess){
//            set_last_ibvs_update_time(time_now);
//            struct FloatVect3 imgcoord[4], F;
//            for (int i = 0; i < nb; i++){
//                imgcoord[i].y = (coord[2*i]-jevois_cx)/jevois_fx;
//                imgcoord[i].x = -(coord[2*i+1]-jevois_cy)/jevois_fy;
//                imgcoord[i].z = 1.;
//            }
//            compute_ibvs_F(imgcoord, &normal, &vartheta, &F);
//            nodetection_count = 0;
//            set_detection_status(true);
//            set_desired_force_ibvs(F.x, F.y, F.z);
//        }else{
//            LOG_INFO("height too low\n");
//        }
    }else if (type == JEVOIS_MSG_F3) {
        static unsigned counter = 0;
        struct timeval now;
        gettimeofday(&now, NULL);
        double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
        struct FloatVect3 normal_gt, vartheta_gt;
        bool gtsuccess = compute_gt(&normal_gt, &vartheta_gt);
        if (gtsuccess){
            struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();
            struct FloatVect3 posb, posned, velb, velned;
            posned.x = stateGetPositionNed_f()->x;
            posned.y = stateGetPositionNed_f()->y;
            posned.z = stateGetPositionNed_f()->z;
            float_rmat_vmult(&posb, Rmat, &posned);

            velned.x = stateGetSpeedNed_f()->x;
            velned.y = stateGetSpeedNed_f()->y;
            velned.z = stateGetSpeedNed_f()->z;
            float_rmat_vmult(&velb, Rmat, &velned);
            set_last_ibvs_update_time(time_now);
            struct FloatVect3 imgcoord[4], F, imgcoordAvg, desiredb_tag, desiredb, b;
            imgcoordAvg.x = imgcoordAvg.y = imgcoordAvg.z = 0.;
            for (int i = 0; i < 4; i++){
                imgcoord[i].y = (coord[3*i]-jevois_cx)/jevois_fx;
                imgcoord[i].x = -(coord[3*i+1]-jevois_cy)/jevois_fy;
                imgcoord[i].z = 1.;
                imgcoordAvg.x += imgcoord[i].x;
                imgcoordAvg.y += imgcoord[i].y;
                imgcoordAvg.z += imgcoord[i].z;
            }
            imgcoordAvg.x /= 4;
            imgcoordAvg.y /= 4;
            imgcoordAvg.z /= 4;

            struct FloatVect3 normal, vartheta;
            const float precision = 1000.f;
            normal.y = ((float)coord[3*4])/precision;
            normal.x = -((float)coord[3*4+1])/precision;
            normal.z = ((float)coord[3*4+2])/precision;

            vartheta.y = ((float)coord[3*5])/precision;
            vartheta.x = -((float)coord[3*5+1])/precision;
            vartheta.z = ((float)coord[3*5+2])/precision;
            const float desired_scale = 1.;
            desiredb_tag.y = desired_scale*((float)coord[3*6])/precision;
            desiredb_tag.x = -desired_scale*((float)coord[3*6+1])/precision;
            desiredb_tag.z = desired_scale*((float)coord[3*6+2])/precision;

            compute_desiredb(&desiredb);
//            desiredb.x = 0.;
//            desiredb.y = 0.;
//            desiredb.z = 5.;

//            if (counter++ %5 == 0)

            LOG_INFO("imgcoord: %f %f %f %f %f %f %f %f\n",
                     imgcoord[0].x, imgcoord[0].y,
                     imgcoord[1].x, imgcoord[1].y,
                     imgcoord[2].x, imgcoord[2].y,
                     imgcoord[3].x, imgcoord[3].y);
            LOG_INFO("n_gt: %f %f %f n_est: %f %f %f\n",
                    normal_gt.x, normal_gt.y, normal_gt.z, normal.x, normal.y, normal.z);
            LOG_INFO("vartheta_gt: %f %f %f vartheta_est: %f %f %f\n",
                    vartheta_gt.x, vartheta_gt.y, vartheta_gt.z, vartheta.x, vartheta.y, vartheta.z);
            LOG_INFO("desiredb: %f %f %f desiredb_tag: %f %f %f\n",
                    desiredb.x, desiredb.y, desiredb.z, desiredb_tag.x, desiredb_tag.y, desiredb_tag.z);
            LOG_INFO("posned: %f %f %f velned: %f %f %f\n", posb.x, posb.y, posb.z, velb.x, velb.y, velb.z);
//                LOG_INFO("imgcoordAvg: %f %f %f desiredb: %f %f %f\n", imgcoordAvg.x, imgcoordAvg.y, imgcoordAvg.z, desiredb.x, desiredb.y, desiredb.z);

            fprintf(fpibvs_out, "%f %f %f %f %f %f %f %f %f", time_now,
                    -jevois_fy*imgcoord[0].x + jevois_cy, jevois_fx*imgcoord[0].y + jevois_cx,
                    -jevois_fy*imgcoord[1].x + jevois_cy, jevois_fx*imgcoord[1].y + jevois_cx,
                    -jevois_fy*imgcoord[2].x + jevois_cy, jevois_fx*imgcoord[2].y + jevois_cx,
                    -jevois_fy*imgcoord[3].x + jevois_cy, jevois_fx*imgcoord[3].y + jevois_cx);

//            fprintf(fpibvs_out, "%f %f %f %f %f %f %f %f %f", time_now,
//                    imgcoord[0].x, imgcoord[0].y,
//                    imgcoord[1].x, imgcoord[1].y,
//                    imgcoord[2].x, imgcoord[2].y,
//                    imgcoord[3].x, imgcoord[3].y);
//            compute_ibvs_F(imgcoord, &normal, &vartheta,
//                    &F, &desiredb, &b);
            compute_ibvs_F(imgcoord, &normal, &vartheta,
                           &F, &desiredb_tag, &b);
//            compute_ibvs_F(imgcoord, &normal_gt, &vartheta_gt,
//                           &F, &desiredb, &b);

            fprintf(fpibvs_out, " %f %f %f %f %f %f",
                    normal_gt.x, normal_gt.y, normal_gt.z, normal.x, normal.y, normal.z);

            fprintf(fpibvs_out, " %f %f %f %f %f %f",
                    vartheta_gt.x, vartheta_gt.y, vartheta_gt.z, vartheta.x, vartheta.y, vartheta.z);

            fprintf(fpibvs_out, " %f %f %f %f %f %f",
                    desiredb_tag.x, desiredb_tag.y, desiredb_tag.z,
                    b.x, b.y, b.z);

            fprintf(fpibvs_out, " %f %f %f %f %f %f\n",
                    posb.x, posb.y, posb.z,
                    F.x, F.y, F.z);

//            compute_ibvs_F(imgcoord, &normal_gt, &vartheta_gt, &F, &desiredb);
            nodetection_count = 0;
            set_detection_status(true);
            set_desired_force_ibvs(F.x, F.y, F.z);
        }else{
            LOG_INFO("height too low\n");
        }
    }else if(type == 10){
        nodetection_count++;
        set_detection_status(false);
//        if (nodetection_count > 3){
//            set_detection_status(false);
//        }
        LOG_INFO("no detections nb: %d coord: %d nodetection: %d\n", nb, coord[0], nodetection_count);
    }else if(type == 20){
        LOG_INFO("no detections\n");
    }
}


void HeadingWorldForce2Att(float Fx, float Fy, float Fz,
                        float *roll, float *pitch,
                        float *U){
    float a = mg - Fz;
    *U = sqrt(a*a + Fx*Fx + Fy*Fy);
    *roll = asin(Fy/(*U));
    *pitch = asin(-Fx/((*U)*cos(*roll)));
}

void BodyForce2Att(float Fx, float Fy, float Fz,
        float *roll, float *pitch,
        float *U){
    *pitch = asin(-Fx/mg);
    *roll = asin(Fy/(mg*cos(*pitch)));
    *U = mg*cos(*roll)*cos(*pitch) - Fz;
}

bool checkftdi(){
    return system("test -e /dev/ttyUSB0") == 0;
}

double get_last_ibvs_update_time(void){
    pthread_mutex_lock (&last_ibvs_update_time_mutex);
    double local_last_ibvs_update_time = last_ibvs_update_time;
    pthread_mutex_unlock(&last_ibvs_update_time_mutex);
    return local_last_ibvs_update_time;
}

void set_last_ibvs_update_time(double update_time){
    pthread_mutex_lock(&last_ibvs_update_time_mutex);
    last_ibvs_update_time = update_time;
    pthread_mutex_unlock(&last_ibvs_update_time_mutex);
}

bool get_detection_status(void){
    pthread_mutex_lock (&detection_status_mutex);
    bool local_detection_status = detection_status;
    pthread_mutex_unlock(&detection_status_mutex);
    return local_detection_status;
}

void set_detection_status(bool status){
    pthread_mutex_lock(&detection_status_mutex);
    detection_status = status;
    pthread_mutex_unlock(&detection_status_mutex);
}

void set_desired_force_ibvs(float Fx, float Fy, float Fz){
    pthread_mutex_lock(&ibvs_force_mutex);
    desired_force_ibvs.x = Fx;
    desired_force_ibvs.y = Fy;
    desired_force_ibvs.z = Fz;
    pthread_mutex_unlock(&ibvs_force_mutex);
}

void get_desired_force_ibvs(float *Fx, float *Fy, float *Fz){
    pthread_mutex_lock(&ibvs_force_mutex);
    *Fx = desired_force_ibvs.x;
    *Fy = desired_force_ibvs.y;
    *Fz = desired_force_ibvs.z;
    pthread_mutex_unlock(&ibvs_force_mutex);
}