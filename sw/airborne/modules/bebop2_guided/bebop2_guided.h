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
 * @file "modules/bebop2_guided/bebop2_guided.h"
 * @author zsk
 * guided mode for bebop2
 */

#ifndef BEBOP2_GUIDED_H
#define BEBOP2_GUIDED_H
#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "generated/airframe.h"
extern int trajectory_guided_mode;
extern void bebop2_guided_init(void);
extern void bebop2_guided_periodic(void);
#endif

