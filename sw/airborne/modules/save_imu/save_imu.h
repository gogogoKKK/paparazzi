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
 * @file "modules/save_imu/save_imu.h"
 * @author zsk
 * save imu data to a file
 */

#ifndef SAVE_IMU_H
#define SAVE_IMU_H
#include <stdbool.h>
extern bool save_imu_record;
extern void save_imu_init(void);

// extern void save_imu_register(void);
// extern void save_imu_periodic();

#endif

