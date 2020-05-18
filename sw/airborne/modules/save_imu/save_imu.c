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
 * @file "modules/save_imu/save_imu.c"
 * @author zsk
 * save imu data to a file
 */
#ifndef IMU_SAVE_PATH
#define IMU_SAVE_PATH /data/video/images
#endif

#ifndef VIDEO_CAPTURE_JPEG_QUALITY
#define VIDEO_CAPTURE_JPEG_QUALITY 99
#endif

#ifndef VIDEO_CAPTURE_FPS
#define VIDEO_CAPTURE_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#include "modules/save_imu/save_imu.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>
# include "pthread.h"

#include "modules/computer_vision/video_capture.h"
#include "modules/computer_vision/cv.h"

#include "lib/encoding/jpeg.h"
#include "state.h"
// Note: this define is set automatically when the video_exif module is included,
// and exposes functions to write data in the image exif headers.
#if JPEG_WITH_EXIF_HEADER
#include "lib/exif/exif_module.h"
#endif

static abi_event imu_ev;
static abi_event imu_lowpass_ev;
pthread_mutex_t mutex ;

bool save_imu_record = false;
bool save_imu_record_start = false;
bool save_img_record = false;
bool save_imu_record_lowpass = false;
int save_img_jump = 2;
FILE *fpimu = NULL;
FILE *fpimu_lowpass = NULL;
static char save_imu_dir[256];
static char save_img_dir[256];

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro,
                    struct Int32Vect3 *accel);

void save_imu_lowpass(uint8_t __attribute__((unused)) sender_id,
                      uint32_t stamp __attribute__((unused)),
                      struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                      struct Int32Vect3 *lp_mag);

void video_capture_save(struct image_t *img);

struct image_t *save_video_capture(struct image_t *img);

void mkdir(char* dir);

bool get_save_img_record(void){
    pthread_mutex_lock (&mutex);
    bool local_save_img_record = save_img_record;
    pthread_mutex_unlock(&mutex);
    return local_save_img_record;
}

bool set_save_img_record(bool record_img){
    pthread_mutex_lock(&mutex);
    save_img_record = record_img;
    pthread_mutex_unlock(&mutex);
}



void save_imu_init(void) {
    AbiBindMsgIMU_INT32(ABI_BROADCAST, &imu_ev, gyro_cb);
//    AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &imu_lowpass_ev, save_imu_lowpass);
    cv_add_to_device(&VIDEO_CAPTURE_CAMERA, save_video_capture, VIDEO_CAPTURE_FPS);
}

//void save_imu_periodic() {
//
//}

void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro,
                    struct Int32Vect3 *accel) {
    if (save_imu_record){
        // create directory to save imu data based on the time
        if (!get_save_img_record()){
            set_save_img_record(true);
            save_imu_record_lowpass = true;
            struct timeval tv;
            struct tm *tm;
            gettimeofday(&tv, NULL);
            tm = localtime(&tv.tv_sec);

            sprintf(save_imu_dir, "%s/%04d_%02d_%02d_%02d_%02d_%02d", STRINGIFY(IMU_SAVE_PATH),
                    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                    tm->tm_hour, tm->tm_min, tm->tm_sec);
            mkdir(save_imu_dir);
//            if (access(save_imu_dir, F_OK)) {
//                char save_dir_cmd[256];
//                sprintf(save_dir_cmd, "mkdir -p %s", save_imu_dir);
//                if (system(save_dir_cmd) != 0) {
//                    printf("[video_capture] Could not create images directory %s.\n", save_imu_dir);
//                    return;
//                }
//            }
            sprintf(save_img_dir, "%s/left", save_imu_dir);
            mkdir(save_img_dir);
            char buf[256];
            sprintf(buf, "%s/imu0.csv", save_imu_dir);
            fpimu = fopen(buf, "w");

//            sprintf(buf, "%s/imu0_lowpass.csv", save_imu_dir);
//            fpimu_lowpass = fopen(buf, "w");
        }
        struct FloatRates gyro_f;
        RATES_FLOAT_OF_BFP(gyro_f, *gyro);
        struct FloatVect3 accel_f;
        ACCELS_FLOAT_OF_BFP(accel_f, *accel);
        struct FloatRMat* pRMat = stateGetNedToBodyRMat_f();
//        printf("%.6f,%.6f,%.6f\n", pRMat->m[2], pRMat->m[5], pRMat->m[8]);
        fprintf(fpimu, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", stamp,
                accel_f.x, accel_f.y, accel_f.z,
                gyro_f.p, gyro_f.q, gyro_f.r,
                pRMat->m[2], pRMat->m[5], pRMat->m[8]);
//        fprintf(fpimu, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", stamp,
//                accel_f.x, accel_f.y, accel_f.z,
//                gyro_f.p, gyro_f.q, gyro_f.r,
//                pRMat->m[0], pRMat->m[1], pRMat->m[2],
//                pRMat->m[3], pRMat->m[4], pRMat->m[5],
//                pRMat->m[6], pRMat->m[7], pRMat->m[8]);
    }else if (get_save_img_record()){
        set_save_img_record(false);
        save_imu_record_lowpass = false;
        fclose(fpimu);
    }

}

struct image_t *save_video_capture(struct image_t *img)
{
    // If take_shot bool is set, save the image
    static unsigned int imgcount = 0;
    if (get_save_img_record() /*&& imgcount++%save_img_jump == 0*/){
        video_capture_save(img);
    }
    // No modification to image
    return NULL;
}

void video_capture_save(struct image_t *img)
{
    // Declare storage for image location
    char save_name[256];

    // Generate image filename from image timestamp
    sprintf(save_name, "%s/%u.jpg", save_img_dir, img->pprz_ts);
    printf("[save_imu] Saving image to %s.\n", save_name);

    // Create jpg image from raw frame
    struct image_t img_jpeg;
    image_create(&img_jpeg, img->w, img->h, IMAGE_JPEG);
    jpeg_encode_image(img, &img_jpeg, VIDEO_CAPTURE_JPEG_QUALITY, true);

#if JPEG_WITH_EXIF_HEADER
    write_exif_jpeg(save_name, img_jpeg.buf, img_jpeg.buf_size, img_jpeg.w, img_jpeg.h);
#else
    // Open file
    FILE *fp = fopen(save_name, "w");
    if (fp == NULL) {
        printf("[save_imu] Could not write shot %s.\n", save_name);
        return;
    }
    // Save it to the file and close it
    fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
    fclose(fp);
#endif
    // Free image
    image_free(&img_jpeg);
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

//void save_imu_lowpass(uint8_t __attribute__((unused)) sender_id,
//                uint32_t stamp __attribute__((unused)),
//                struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
//                struct Int32Vect3 *lp_mag)
//{
//    if (save_imu_record_lowpass) {
//        /* convert to float */
//        struct FloatRates gyro_f;
//        RATES_FLOAT_OF_BFP(gyro_f, *lp_gyro);
//        struct FloatVect3 accel_f;
//        ACCELS_FLOAT_OF_BFP(accel_f, *lp_accel);
//        struct FloatVect3 mag_f;
//        MAGS_FLOAT_OF_BFP(mag_f, *lp_mag);
//        fprintf(fpimu, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", stamp,
//                accel_f.x, accel_f.y, accel_f.z,
//                gyro_f.p, gyro_f.q, gyro_f.r,
//                mag_f.x, mag_f.y, mag_f.z);
//    }
//}
//void save_imu_register(void)
//{
//    save_imu_init();
//    /*
//     * Subscribe to scaled IMU measurements and attach callbacks
//     */
//    AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev, gyro_cb);
//    AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev, accel_cb);
//}


