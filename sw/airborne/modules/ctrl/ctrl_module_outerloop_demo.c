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
#include <math.h>
// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;
float ctrl_outerloop_kp = 1., ctrl_outerloop_kv = .3, ctrl_outerloop_kh = 20000., ctrl_outerloop_kvz = 12000.;
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

    float sp_yaw = -RadOfDeg(90.), sp_h = -1.;

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

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

