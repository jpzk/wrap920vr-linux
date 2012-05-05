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

#include "attitude_sensor.h"

#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

ATTITUDE_SENSOR* attitude_sensor_new() {
    ATTITUDE_SENSOR *self = (ATTITUDE_SENSOR *) malloc(sizeof(ATTITUDE_SENSOR));
   
    /**
     * Try to open the hidraw device for reading the raw data. 
     * In future using libusb would be more elegant. 
     */
    self->file_device = open(
        ATTITUDE_SENSOR_HIDRAW,
        O_RDWR | O_NONBLOCK);

    if(self->file_device < 0) {
        LOG("Could not open device.");
        return(NULL);
    }	
	
    self->zero_angles.yaw = 0.0;
    self->zero_angles.pitch = 0.0;
    self->zero_angles.roll = 0.0;
    self->current_angles.yaw = 0.0;
    self->current_angles.pitch = 0.0;
    self->current_angles.roll = 0.0;
    self->use_yaw = true;
    self->use_pitch = true;
    self->use_roll = true;

    /**
     * Checking if config file already exists. If config already 
     * exists then read configuration, otherwise calibrate and 
     * write configuration.
     */
    FILE *config = fopen(ATTITUDE_SENSOR_CONFIG_FILE, "r");
    if(config) {
        fclose(config);
        attitude_sensor_read_config(self);
    } else {
        self->bias_gyro = attitude_sensor_estimate_gyro_bias(self);
        attitude_sensor_calibrate(self);
        attitude_sensor_write_config(self);
    }

    /**
     * Clearing the ringbuffers, which are used for smoothing the 
     * sensor values
     */
    self->ringbuffer_acc_pitch.pointer = 0;
    self->ringbuffer_acc_roll.pointer = 0;
    self->current_acc_pitch = 0.0;
    self->current_acc_roll = 0.0;

    int i = 0;
    for(;i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++) {
        self->ringbuffer_acc_pitch.measures[i] = 0.0;
        self->ringbuffer_acc_roll.measures[i] = 0.0;
    }

    /**
     * Everything went fine.
     */
    self->vuzix_connected = true;
  
    LOG("AttitudeSensor instantiated.");
    return(self);
}

void attitude_sensor_delete(ATTITUDE_SENSOR *self) {
    free(self);
}

HEAD_DIRECTION attitude_sensor_get_head(ATTITUDE_SENSOR *self) {
    return(self->head);
}

void attitude_sensor_read_config(ATTITUDE_SENSOR *self) {
    FILE *config = fopen(ATTITUDE_SENSOR_CONFIG_FILE, "r")

    fscanf(config, "%i;%i;%i\n", 
        &(self->bias_gyro.x),
        &(self->bias_gyro.y),
        &(self->bias_gyro.z));

    fscanf(config, "%i;%i;%i\n",
        &(self->calib_mag_min.x),
        &(self->calib_mag_min.y),
        &(self->calib_mag_min.z));

    fscanf(config, "%i;%i;%i\n",
        &(self->calib_mag_max.x),
        &(self->calib_mag_max.y),
        &(self->calib_mag_max.z));

    fscanf(config, "%i;%i;%i\n",
        &(self->calib_acc_min.x),
        &(self->calib_acc_min.y),
        &(self->calib_acc_min.z));

    fscanf(config, "%i;%i;%i\n",
        &(self->calib_acc_max.x),
        &(self->calib_acc_max.y),
        &(self->calib_acc_max.z));

    fclose(config);        
}

void attitude_sensor_write_config(ATTITUDE_SENSOR *self) {
    FILE *config = fopen(ATTITUDE_SENSOR_CONFIG_FILE, "w")
    
    fprintf(config, "%i;%i;%i\n", 
        self->bias_gyro.x,
        self->bias_gyro.y,
        self->bias_gyro.z);

    fprintf(config, "%i;%i;%i\n",
        self->calib_mag_min.x,
        self->calib_mag_min.y,
        self->calib_mag_min.z);

    fprintf(config, "%i;%i;%i\n",
        self->calib_mag_max.x,
        self->calib_mag_max.y,
        self->calib_mag_max.z);

    fprintf(config, "%i;%i;%i\n",
        self->calib_acc_min.x,
        self->calib_acc_min.y,
        self->calib_acc_min.z);

    fprintf(config, "%i;%i;%i\n",
        self->calib_acc_max.x,
        self->calib_acc_max.y,
        self->calib_acc_max.z);

    fclose(config);        
}

void attitude_sensor_receive(ATTITUDE_SENSOR *self) {
    self->bytes_read = read(
        self->file_device, 
        self->buf, 
        ATTITUDE_SENSOR_BUFFERSIZE);
	
    if (self->bytes_read < 0) {
    } else {
        memcpy(
            &(self->sensdata), 
            &(self->buf[2]),  //offset
            sizeof(unsigned char) * 24);
        self->parsed = attitude_sensor_parse_data(self);
    }
}

ANGLES calculate_angles( 
    ANGLES *current_angles,		
    IWRSENSOR_PARSED_F *normalized_mag_sensor_data, 
    IWRSENSOR_PARSED_F *normalized_acc_sensor_data, 
    IWRSENSOR_PARSED *normalized_gyr_sensor_data, 
    ANGLES *current_gyro,
    RINGBUFFER *ringbuffer_acc_pitch,
    RINGBUFFER *ringbuffer_acc_roll,
    float *current_acc_pitch,
    float *current_acc_roll) {

    ANGLES ret_val;

    ret_val.yaw = calculate_yaw(
        normalized_mag_sensor_data, 
        normalized_gyr_sensor_data, 
        current_gyro,
        &(current_angles->yaw), 
        current_acc_pitch, 
        current_acc_roll);

    ret_val.pitch = calculate_pitch(
        &(current_angles->pitch), 
        normalized_acc_sensor_data, 
        normalized_gyr_sensor_data, 
        current_gyro, 
        ringbuffer_acc_pitch, 
        current_acc_pitch);
	
    ret_val.roll = calculate_roll(
        &(current_angles->roll),
        normalized_acc_sensor_data, 
        normalized_gyr_sensor_data, 
        current_gyro, 
        ringbuffer_acc_roll, 
        current_acc_roll);
	
    return(ret_val);
}

/*
 * Calculate the pitch angle from accelerometer and gyroscope input.
 * Side-effects: be aware that this function changes the ringbuffer for
 * the accelerometer in order to filter the data.
 */
float calculate_pitch(
    float *current_pitch,
    IWRSENSOR_PARSED_F *normalized_acc_sensor_data,
    IWRSENSOR_PARSED *normalized_gyr_sensor_data, 
    ANGLES *current_gyro,
    RINGBUFFER *ringbuffer_acc_pitch,
    float *current_acc_pitch) {

    float ret_val;
    float current_acc_pitch_val = *current_acc_pitch;

    //Filter Acc Data
    ringbuffer_acc_pitch->measures[ringbuffer_acc_pitch->pointer] = atan2(
        normalized_acc_sensor_data->x, 
        sqrt(normalized_acc_sensor_data->z * normalized_acc_sensor_data->z + 
        normalized_acc_sensor_data->y * normalized_acc_sensor_data->y)) / M_PI; 

    float acc_avg = 0.0;

    unsigned int i = 0;
    for(; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){	
        unsigned int current_index = (ringbuffer_acc_pitch->pointer + i) % 
            ATTITUDE_SENSOR_RINGBUFFER_SIZE;
		
        float coefficient = geometric_distribution(
            ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY, i);

        acc_avg += coefficient * ringbuffer_acc_pitch->measures[current_index];	
    }

    ringbuffer_acc_pitch->pointer = ringbuffer_acc_pitch->pointer + 1;
	
    if(ringbuffer_acc_pitch->pointer == ATTITUDE_SENSOR_RINGBUFFER_SIZE) {
        ringbuffer_acc_pitch->pointer = 0; 
    }
		
	//Gyro is difference, acc is absolute position
    acc_avg *= 90.0;
    current_acc_pitch_val = acc_avg;

	//Add gyro to currentPitch
    *current_pitch = *current_pitch + 
        normalized_gyr_sensor_data->y * (90.0/32768.0);

    float possible_error_pitch = current_acc_pitch_val - *current_pitch;

    if(possible_error_pitch > 2.0 || possible_error_pitch < -2.0){
        *current_pitch = *current_pitch + 0.05 * possible_error_pitch;
    }

    ret_val = *current_pitch;
    return(ret_val); 
}

float calculate_roll(
    float *current_roll,
    IWRSENSOR_PARSED_F *normalized_acc_sensor_data,
    IWRSENSOR_PARSED *normalized_gyr_sensor_data, 
    ANGLES *current_gyro,
    RINGBUFFER *ringbuffer_acc_roll,
    float *current_acc_roll) {

    float ret_val;

    //Filter Acc Data
    ringbuffer_acc_roll->measures[ringbuffer_acc_roll->pointer] = atan2(	
            sqrt(normalized_acc_sensor_data->x * normalized_acc_sensor_data->x + 
            normalized_acc_sensor_data->z * normalized_acc_sensor_data->z),
            normalized_acc_sensor_data->y) / M_PI; 
	
    float acc_avg=0.0;

    unsigned int i = 0;
    for(; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){

        unsigned int current_index = (ringbuffer_acc_roll->pointer + i) 
            % ATTITUDE_SENSOR_RINGBUFFER_SIZE;
	
        float coefficient = geometric_distribution(
            ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY,i);
		
        acc_avg += coefficient * ringbuffer_acc_roll->measures[current_index];	
    }

    ringbuffer_acc_roll->pointer = ringbuffer_acc_roll->pointer + 1;
	
    if(ringbuffer_acc_roll->pointer == ATTITUDE_SENSOR_RINGBUFFER_SIZE){
        ringbuffer_acc_roll->pointer = 0;	
    }
		
    *current_acc_roll = acc_avg * 90.0;

	//Add gyro to currentPitch
    *current_roll = *current_roll + 
        normalized_gyr_sensor_data->z * 0.5 * (180.0/32768.0);
    
    float possible_error_roll = *current_acc_roll - *current_roll;

    if(possible_error_roll > 1.0) {
        *current_roll = *current_roll + 0.05 * possible_error_roll;
    } else if(possible_error_roll < -1.0) {
        *current_roll = *current_roll + 0.05 * possible_error_roll;
    }

    ret_val = *current_roll;
    return ret_val; 
}

float calculate_yaw(
    IWRSENSOR_PARSED_F *normalized_mag_sensor_data, 
    IWRSENSOR_PARSED *normalized_gyr_sensor_data, 
    ANGLES *current_gyro,
    float *current_yaw,
    float *current_pitch,
    float *current_roll) {    

    static double last_mag_yaw=0.0;
    IWRSENSOR_PARSED_F mag = *normalized_mag_sensor_data;
    float ret_val;
    float mag_yaw;
	
    float gyr_diff = normalized_gyr_sensor_data->x * 0.2 * (180.0 / 32768.0);

    double xh=mag.x;
    double yh=mag.y;
    double length = sqrt(xh * xh + yh * yh);
    xh = xh / length;
    yh = yh / length;
    mag_yaw = atan2(xh, yh); 
    mag_yaw *= 0.2 * (180.0 / 32768.0);
	
    float mag_diff = mag_yaw - last_mag_yaw;
    float possible_error_yaw = gyr_diff - mag_diff;

    if(possible_error_yaw > 1.0){
        ret_val = current_gyro->yaw + gyr_diff - 0.05 * possible_error_yaw;
    } else if(possible_error_yaw < -1.0) {
        ret_val = current_gyro->yaw + gyr_diff + 0.05 * possible_error_yaw;
    } else {
        ret_val = current_gyro->yaw + gyr_diff;
    }

    current_gyro->yaw = ret_val;
    return(ret_val);
}

float geometric_distribution(float p, int k){
    return p * pow((1-p), k-1);
}

void attitude_sensor_calibrate(ATTITUDE_SENSOR *self) {
    attitude_sensor_receive(self);
	
    self->calib_mag_min = self->parsed.mag_sensor;
    self->calib_mag_max = self->parsed.mag_sensor;
    self->calib_acc_min = self->parsed.acc_sensor;
    self->calib_acc_max = self->parsed.acc_sensor;
	
    int16_t *ptr = (int16_t *) &(self->parsed);
    int16_t *ptr_mag_min = (int16_t *) &(self->calib_mag_min);
    int16_t *ptr_mag_max = (int16_t *) &(self->calib_mag_max);
    int16_t *ptr_acc_min = (int16_t *) &(self->calib_acc_min);
    int16_t *ptr_acc_max = (int16_t *) &(self->calib_acc_max);

    int i = 1;
    for(; i < 250; i++) {//TODO define
		
        attitude_sensor_receive(self);

        int k = 0;
        for(; k < 3; k++) {
            if(ptr_mag_min[k] > ptr[k]){
                ptr_mag_min[k] = ptr[k];
            } else if(ptr_mag_max[k] < ptr[k]){
                ptr_mag_max[k] = ptr[k];
            }
        }

        k = 3;
        for(; k < 6; k++) {
            if(ptr_acc_min[k - 3] > ptr[k]) {
                ptr_acc_min[k - 3] = ptr[k];
            } else if(ptr_acc_max[k - 3] < ptr[k]) {
                ptr_acc_max[k - 3] = ptr[k];
            }
        }

        //usleep(50000); //50 millis

        printf("calibMin: %i %i %i %i %i %i \n", 
            self->calib_mag_min.x,
            self->calib_mag_min.y,
            self->calib_mag_min.z,
            self->calib_acc_min.x,
            self->calib_acc_min.y,
            self->calib_acc_min.z);

        printf("calibMax: %i %i %i %i %i %i \n", 
            self->calib_mag_max.x,
            self->calib_mag_max.y,
            self->calib_mag_max.z,
            self->calib_acc_max.x,
            self->calib_acc_max.y,
            self->calib_acc_max.z);
    }
}

/**
 * Read data, glasses must lie on the desk, and must not be moved.
 */
IWRSENSOR_PARSED attitude_sensor_estimate_gyro_bias(ATTITUDE_SENSOR *self) {
    IWRSENSOR_PARSED ret_val;
    long sums[3]={0,0,0};	

    int i = 0;
    for(; i < 4000; i++) {//TODO define
        attitude_sensor_receive(self);
        //int16_t **ptr = (int16_t **) &parsed; //TODO wieso nicht?
        int16_t *ptr = (int16_t *) &(self->parsed);

        int k = 6;
        for(; k < 9; k++) {
            sums[k - 6] += (long) ptr[k];
        }
    }

    // calculate average
    ret_val.x = ((float) sums[0])/4000.0;
    ret_val.y = ((float) sums[1])/4000.0;
    ret_val.z = ((float) sums[2])/4000.0;
    LOG("Gyroscope Bias x: %d", ret_val.x);
    LOG("Gyroscope Bias y: %d", ret_val.y);
    LOG("Gyroscope Bias z: %d", ret_val.z);
    LOG("Calibrate Gyro done");
    return(ret_val);
}	

float normalize_value(
    int16_t *min, 
    int16_t *max,
    int16_t *value){

    float val = (float) *value;

    if(value < min) val = (float) *min;
	if(value > max) val = (float) *max;
	
	float ret_val=
		2.0f *
		((val - (float) *min))/
		((float) *max - (float) *min)
		-1.0f;

	return ret_val;
}

IWRSENSOR_PARSED normalize_gyro(
    IWRSENSOR_PARSED *bias_gyro, 
	IWRSENSOR_PARSED *sensor){

	IWRSENSOR_PARSED ret_val;	
	ret_val.x = (*sensor).x - (*bias_gyro).x;
	ret_val.y = (*sensor).y - (*bias_gyro).y;
	ret_val.z = (*sensor).z - (*bias_gyro).z;
	
    return ret_val;
}

IWRSENSOR_PARSED_F normalize_sensor(
    IWRSENSOR_PARSED *calib_min,  
    IWRSENSOR_PARSED *calib_max, 
    IWRSENSOR_PARSED *sensor) {

    IWRSENSOR_PARSED_F ret_val;
	
    ret_val.x = normalize_value(
        &((*calib_min).x), 
        &((*calib_max).x), 
        &((*sensor).x));

	ret_val.y = normalize_value(
        &((*calib_min).y),
        &((*calib_max).y),
        &((*sensor).y));

	ret_val.z = normalize_value(
        &((*calib_min).z),
        &((*calib_max).z), 
        &((*sensor).z));

    return ret_val;
}

IWRSENSDATA_PARSED attitude_sensor_parse_data(ATTITUDE_SENSOR *self) {
    IWRSENSDATA_PARSED ret;
    unsigned char *ptr_data = (unsigned char*) &(self->sensdata);
    signed short *ptr_ret = (signed short*) &ret;
    
    int i = 0;
    int j = 0;
    for(; i < 12; i += 2,  j++) { //mag and acc
        unsigned char *lsb = ptr_data + i;
        unsigned char *msb = ptr_data + i + 1;
        *(ptr_ret + j) = (((unsigned short) *msb << 8) | (unsigned short) *lsb);
    }

    i = 18;
    j = 6;
//	for(int i = 12, j=6; i < 18; i+=2,  j++) { //high bandwidth gyro
    for(; i < 24; i+=2,  j++) { //low bandwidth gyro
        unsigned char *lsb = ptr_data + i;
        unsigned char *msb = ptr_data + i + 1;
        *(ptr_ret + j) = (((unsigned short) *msb << 8) | (unsigned short) *lsb);
    } 
    return ret;
}

void attitude_sensor_timer_proc(ATTITUDE_SENSOR *self) {
    if(!self->vuzix_connected){
        return;
    }
    attitude_sensor_receive(self);

    IWRSENSOR_PARSED_F normalized_mag_sensor_data =
        normalize_sensor(
            &(self->calib_mag_min),
            &(self->calib_mag_max), 
            &(self->parsed.mag_sensor));
	
    IWRSENSOR_PARSED_F normalized_acc_sensor_data = 
        normalize_sensor(
            &(self->calib_acc_min), 
            &(self->calib_acc_max), 
            &(self->parsed.acc_sensor));

    IWRSENSOR_PARSED normalized_gyr_sensor_data =    
        normalize_gyro(
            &(self->bias_gyro), 
            &(self->parsed.gyro_sensor));	
	
    ANGLES angles = calculate_angles(
        &(self->current_angles),		
	    &(normalized_mag_sensor_data), 
	    &(normalized_acc_sensor_data), 
	    &(normalized_gyr_sensor_data), 
	    &(self->current_gyro), 
	    &(self->ringbuffer_acc_pitch), 
	    &(self->ringbuffer_acc_roll), 
	    &(self->current_acc_pitch),
	    &(self->current_acc_roll));

	if(self->use_yaw)
		self->head.yaw_deg = angles.yaw - self->zero_angles.yaw; 
	
	if(self->use_pitch)
		self->head.pitch_deg = angles.pitch - self->zero_angles.pitch; 
	
	if(self->use_roll)
		self->head.roll_deg = angles.roll - self->zero_angles.roll;
}

void attitude_sensor_reset_head(ATTITUDE_SENSOR *self) {
	self->zero_angles.yaw = self->current_gyro.yaw;
	self->zero_angles.pitch = self->current_angles.pitch;
	self->zero_angles.roll = self->current_angles.roll;
	
	printf("Zeroes: %f || %f || %f\n", self->zero_angles.yaw, 
	self->zero_angles.pitch, self->zero_angles.roll);
}

void attitude_sensor_toggle_use_yaw(ATTITUDE_SENSOR *self) {
	self->use_yaw = (!self->use_yaw);
}	

void attitude_sensor_toggle_use_pitch(ATTITUDE_SENSOR *self) {
	self->use_pitch = (!self->use_pitch);
}	

void attitude_sensor_toggle_use_roll(ATTITUDE_SENSOR *self) {
	self->use_roll = (!self->use_roll);
}	

