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
#include "pthread.h"

#include "modules/computer_vision/video_capture.h"
#include "modules/computer_vision/cv.h"
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include "lib/encoding/jpeg.h"
#include "state.h"
// Note: this define is set automatically when the video_exif module is included,
// and exposes functions to write data in the image exif headers.
#if JPEG_WITH_EXIF_HEADER
#include "lib/exif/exif_module.h"
#endif

static abi_event imu_ev;
static abi_event jevois_ev;
static abi_event imu_lowpass_ev;
pthread_mutex_t mutex;

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

static void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
        uint8_t nb, int16_t * coord, uint16_t * dim,
        struct FloatQuat quat, char * extra);

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
    AbiBindMsgJEVOIS_MSG(CAM_JEVOIS_ID, &jevois_ev, jevois_msg_event);
//    AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &imu_lowpass_ev, save_imu_lowpass);
    cv_add_to_device(&VIDEO_CAPTURE_CAMERA, save_video_capture, VIDEO_CAPTURE_FPS);
//    set_serial();
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
        fprintf(fpimu, "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", stamp,
                accel_f.x, accel_f.y, accel_f.z,
                gyro_f.p, gyro_f.q, gyro_f.r);
//        struct FloatRMat* pRMat = stateGetNedToBodyRMat_f();
////        printf("%.6f,%.6f,%.6f\n", pRMat->m[2], pRMat->m[5], pRMat->m[8]);
//        fprintf(fpimu, "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", stamp,
//                accel_f.x, accel_f.y, accel_f.z,
//                gyro_f.p, gyro_f.q, gyro_f.r,
//                pRMat->m[2], pRMat->m[5], pRMat->m[8]);
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

void jevois_msg_event(uint8_t sender_id, uint8_t type, char * id,
                      uint8_t nb, int16_t * coord, uint16_t * dim,
                      struct FloatQuat quat, char * extra){

    struct timeval now;
    gettimeofday(&now, NULL);
    double time_now = (double)now.tv_sec + (double)now.tv_usec*1e-6;
//    int timeint = time_now*1e3;
//    printf("guided %d %d %d %f\n", now.tv_sec, now.tv_usec, time_now_ms, time_now);
    printf("[receive] sender_id: %f %u type %u nb: %u\n", time_now, sender_id, type, nb);
    if (type == 23){
        for (int i = 0; i<nb; i++){
            printf(" coord %d, %d\n", coord[2*i], coord[2*i+1]);
        }
    }

//    printf("\n");
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
//    printf(save_name);
    // Save it to the file and close it
//    fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
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
//void set_serial(){
//    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
//    int serial_port = open("/dev/ttyUSB0", O_RDWR);
//
//    // Create new termios struc, we call it 'tty' for convention
//    struct termios tty;
//    memset(&tty, 0, sizeof tty);
//
//    // Read in existing settings, and handle any error
//    if(tcgetattr(serial_port, &tty) != 0) {
//        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
//    }
//
//    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
//    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
//    tty.c_cflag |= CS8; // 8 bits per byte (most common)
//    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
//    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
//
//    tty.c_lflag &= ~ICANON;
//    tty.c_lflag &= ~ECHO; // Disable echo
//    tty.c_lflag &= ~ECHOE; // Disable erasure
//    tty.c_lflag &= ~ECHONL; // Disable new-line echo
//    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
//    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
//    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
//
//    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
//    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
//    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
//    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
//
//    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
//    tty.c_cc[VMIN] = 0;
//
//    // Set in/out baud rate to be 9600
//    cfsetispeed(&tty, B115200);
//    cfsetospeed(&tty, B115200);
//
//    // Save tty settings, also checking for error
//    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
//        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
//    }
//
//    // Write to serial port
//    unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
//    write(serial_port, "Hello, world!", sizeof(msg));
//
//    // Allocate memory for read buffer, set size according to your needs
//    char read_buf [256];
//    memset(&read_buf, '\0', sizeof(read_buf));
//
//    // Read bytes. The behaviour of read() (e.g. does it block?,
//    // how long does it block for?) depends on the configuration
//    // settings above, specifically VMIN and VTIME
//    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
//
//    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
//    if (num_bytes < 0) {
//        printf("Error reading: %s", strerror(errno));
//    }
//
//    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
//    // print it to the screen like this!)
//    printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
//    close(serial_port);
//}


