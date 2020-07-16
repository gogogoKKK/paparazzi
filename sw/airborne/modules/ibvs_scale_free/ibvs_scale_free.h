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
 * @file "modules/ibvs_scale_free/ibvs_scale_free.h"
 * @author zsk
 * 
 */

#ifndef IBVS_SCALE_FREE_H
#define IBVS_SCALE_FREE_H
#include <std.h>
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// But re-using an existing altitude-hold controller
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER
extern float ctrl_outerloop_kp, ctrl_outerloop_kv, ctrl_outerloop_kh, ctrl_outerloop_kvz;
extern float ibvs_h_k0, ibvs_h_k1, ibvs_v_k0, ibvs_v_k1;
extern int jevois_start_status;
extern bool use_ibvs;
extern void ibvs_scale_free_init(void);
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

#endif

