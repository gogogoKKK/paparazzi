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
 * @file "modules/motor_mixing_identify/motor_mixing_identify.h"
 * @author zsk
 * 
 */

#ifndef MOTOR_MIXING_IDENTIFY_H
#define MOTOR_MIXING_IDENTIFY_H

extern int motor_TL_ud;
extern int motor_TR_ud;
extern int motor_BR_ud;
extern int motor_BL_ud;
extern void motor_identify_init();
extern void motor_identify_periodic();

#endif

