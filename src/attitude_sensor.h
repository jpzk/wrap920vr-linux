/*
Copyright (c) 2012, Justin Philipp Heinermann and Jendrik Poloczek
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

#ifndef _ATTITUDE_SENSOR_H
#define _ATTITUDE_SENSOR_H

#include <stdlib.h>
#include <stdint.h>

#define ATTITUDE_SENSOR_VENDOR 0x1bae
#define ATTITUDE_SENSOR_PRODUCT 0x014b
#define ATTITUDE_SENSOR_HIDRAW "/dev/vuzix"
#define ATTITUDE_SENSOR_BUFFERSIZE 26
#define ATTITUDE_SENSOR_RINGBUFFER_SIZE 10 
#define ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY 0.5
#define ATTITUDE_SENSOR_CONFIG_FILE "attitudesensor.conf"

#define LOG(string, args...) printf (string"\n", ##args)

typedef struct tag_HEAD_DIRECTION {
    float yawDeg;
    float rollDeg;
    float pitchDeg;
} HEAD_DIRECTION;

typedef struct tag_IWRSENSOR_PARSED {
    int16_t x, y, z;
} IWRSENSOR_PARSED;

typedef struct tag_IWRSENSOR_PARSED_F {
    float x, y, z;
} IWRSENSOR_PARSED_F;

typedef struct tag_IWRSENSDATA_PARSED {
    IWRSENSOR_PARSED mag_sensor, acc_sensor, gyro_sensor;
} IWRSENSDATA_PARSED;

typedef struct tag_IWRSENSOR {
    unsigned char x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
} IWRSENSOR, *PIWRSENSOR;
    
typedef struct tag_IWRSENSDATA {
    IWRSENSOR mag_sensor, acc_sensor, gyro_sensor;
} IWRSENSDATA, *PIWRSENSDATA;

typedef struct tag_ANGLES {
    float yaw, pitch, roll;
} ANGLES;
	
typedef struct tag_RINGBUFFER {
    float measures[ATTITUDE_SENSOR_RINGBUFFER_SIZE]; 
    unsigned int pointer;
} RINGBUFFER;

typedef struct tag_ATTITUDE_SENSOR {
    bool use_yaw, use_pitch, use_roll;
    float current_acc_pitch;
    float current_acc_roll;
    int file_device, bytes_read;
    unsigned char buf[28]; //TODO magic

    HEAD_DIRECTION head_direction;
    static bool vuzix_connected;
	
    IWRSENSDATA sensdata;
    IWRSENSDATA_PARSED parsed;
    IWRSENSOR_PARSED calib_mag_min;
    IWRSENSOR_PARSED calib_mag_max;
    IWRSENSOR_PARSED calib_acc_min;
    IWRSENSOR_PARSED calib_acc_max;
    IWRSENSOR_PARSED bias_gyro;

    ANGLES zero_angles;
    ANGLES current_gyro;
    ANGLES current_acc;
    ANGLES current_angles;

    RINGBUFFER ringbuffer_acc_pitch;
    RINGBUFFER ringbuffer_acc_roll;		
} ATTITUDE_SENSOR;

/**
 * ATTITUDE_SENSOR methods 
 */
ATTITUDE_SENSOR *attitude_sensor_new();
void attitude_sensor_delete(ATTITUDE_SENSOR *self);

void attitude_sensor_timer_proc(ATTITUDE_SENSOR *self);
void attitude_sensor_reset_head(ATTITUDE_SENSOR *self);
void attitude_sensor_get_head(ATTITUDE_SENSOR *self);

void attitude_sensor_toggle_use_yaw(ATTITUDE_SENSOR *self);
void attitude_sensor_toggle_use_pitch(ATTITUDE_SENSOR *self);
void attitude_sensor_toggle_use_roll(ATTITUDE_SENSOR *self);

void attitude_sensor_read_config(ATTITUDE_SENSOR *self, const char *config);
void attitude_sensor_write_config(ATTITUDE_SENSOR *self);

void attitude_sensor_estimate_gyro_bias(ATTITUDE_SENSOR *self);
void attitude_calibrate(ATTITUDE_SENSOR *self);
void attitude_receive(ATTITUDE_SENSOR *self);

IWRSENSDATA_PARSED attitude_sensor_parse_data(ATTITUDE_SENSOR *self);

/**
 * Helper static methods 
 */

float normalize_value(
    int16_t *min, 
    int16_t *max,  
    int16_t *value);
	
IWRSENSOR_PARSED_F normalize_sensor(
    IWRSENSOR_PARSED *calibMin,  
    IWRSENSOR_PARSED *calibMax, 
    IWRSENSOR_PARSED *sensor);
	
IWRSENSOR_PARSED normalize_gyro(
    IWRSENSOR_PARSED *biasGyro, 
    IWRSENSOR_PARSED *sensor );

ANGLES calculate_angles( 
    ANGLES *currentAngles,		
    IWRSENSOR_PARSED_F *normalizedMagSensorData, 
    IWRSENSOR_PARSED_F *normalizedAccSensorData,
    IWRSENSOR_PARSED *normalizedGyrSensorData, 
    ANGLES *currentGyro, 
    RINGBUFFER *ringbufferAccPitch, 
    RINGBUFFER *ringbufferAccRoll, 
    float *currentAccPitch,
    float *currentAccRoll);
	
float calculate_pitch(
    float *currentPitch,		
    IWRSENSOR_PARSED_F *normalizedAccSensorData, 
    IWRSENSOR_PARSED *normalizedGyrSensorData, 
    ANGLES *currentGyro, 
    RINGBUFFER *ringbufferAccPitch, 
    float *currentAccPitch);

float calculate_roll(
    float *currentRoll,		
    IWRSENSOR_PARSED_F *normalizedAccSensorData, 
    IWRSENSOR_PARSED *normalizedGyrSensorData, 
    ANGLES *currentGyro, 
    RINGBUFFER *ringbufferAccRoll, 
    float *currentAccRoll);
	
float calculate_yaw(
    IWRSENSOR_PARSED_F *normalizedMagSensorData, 
    IWRSENSOR_PARSED *normalizedGyrSensorData, 
    ANGLES *currentGyro,
    float *currentYaw,
    float *currentPitch,
    float *currentRoll);

float geometric_distribution(float p, int k);

#endif
