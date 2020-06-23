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
#include "subsystems/abi.h"
// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;

static float jevois_fx = 406.7542857,
jevois_fy = 408.1222019,
jevois_cx = 321.65766064,
jevois_cy = 245.95284275;


static abi_event jevois_ev;
// Settings
float ctrl_outerloop_kp = 10., ctrl_outerloop_kv = 6., ctrl_outerloop_kh = 20000., ctrl_outerloop_kvz = 12000.;

static void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                             uint8_t nb, int16_t * coord, uint16_t * dim,
                             struct FloatQuat quat, char * extra);

void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra){

    struct timeval now;
    gettimeofday(&now, NULL);
    double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
    float imgcoord[4][2];
//    int timeint = time_now*1e3;
//    printf("guided %d %d %d %f\n", now.tv_sec, now.tv_usec, time_now_ms, time_now);
//    printf("[receive] sender_id: %f %u type %u nb: %u\n", time_now, sender_id, type, nb);
    if (type == 23){
        for (int i = 0; i<nb; i++){
            imgcoord[i][0] = (coord[2*i]-jevois_cx)/jevois_fx;
            imgcoord[i][1] = (coord[2*i+1]-jevois_cy)/jevois_fy;
//            printf(" coord %d, %d\n", coord[2*i], coord[2*i+1]);
        }
    }

    float vxned = stateGetSpeedNed_f()->x;
    float vyned = stateGetSpeedNed_f()->y;
    float vzned = stateGetSpeedNed_f()->z;

    float pxned = stateGetPositionNed_f()->x;
    float pyned = stateGetPositionNed_f()->y;
    float pzned = stateGetPositionNed_f()->z;

    struct FloatRMat *Rmat = stateGetNedToBodyRMat_f();

//  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
//  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);
//  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(yaw);
//  stabilization_cmd[COMMAND_THRUST] = 1000;
//  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
//  stabilization_attitude_run(in_flight);

//    printf("\n");
}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
//    AbiBindMsgJEVOIS_MSG(CAM_JEVOIS_ID, &jevois_ev, jevois_msg_event);
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

    printf("inflight: %d pos: %f %f %f vel: %f %f %f yaw: %f\n", in_flight, pxned, pyned, pzned, vxned, vyned, vzned, yaw);

    float maxdeg = 15., thrust_max = 3000;
    float pitch_cmd = (pxrobot*ctrl_outerloop_kp + vxrobot*ctrl_outerloop_kv);
    BoundAbs(pitch_cmd, maxdeg);
    pitch_cmd = RadOfDeg(pitch_cmd);
    float roll_cmd = (-pyrobot*ctrl_outerloop_kp - vyrobot*ctrl_outerloop_kv);
    BoundAbs(roll_cmd, maxdeg);
    roll_cmd = RadOfDeg(roll_cmd);

//    float roll = RadOfDeg(10), pitch = RadOfDeg(0); yaw = 0.;
//    printf("cmd: %f %f %f\n", roll_cmd, pitch_cmd, sp_yaw);
    ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll_cmd);
    ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch_cmd);
    ctrl.cmd.psi = ANGLE_BFP_OF_REAL(sp_yaw);
    float thrust_cmd = (pzned - sp_h)*ctrl_outerloop_kh+(vzned)*ctrl_outerloop_kvz;
    BoundAbs(thrust_cmd, thrust_max);
    stabilization_cmd[COMMAND_THRUST] = thrust_cmd+4000;
//    printf("thrust: %d\n", stabilization_cmd[COMMAND_THRUST]);
    stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
    stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

