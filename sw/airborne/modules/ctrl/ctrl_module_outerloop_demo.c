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
// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;


// Settings
float comode_time = 0;


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
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);
//  float roll = 10.0/57.3;
  float roll = 0.;
  float pitch = 0.0;
  float yaw = 0.;
  motor_mixing.commands[MOTOR_FRONT_LEFT] = 100;
  motor_mixing.commands[MOTOR_FRONT_RIGHT] = 100;
  motor_mixing.commands[MOTOR_BACK_RIGHT] = 100;
  motor_mixing.commands[MOTOR_BACK_LEFT] = 100;

  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(yaw);
  stabilization_cmd[COMMAND_THRUST] = 1000;
  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

