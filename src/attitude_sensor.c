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

#include <math.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>

ATTITUDE_SENSOR* attitude_sensor_new() {
    ATTITUDE_SENSOR *self = (ATTITUDE_SENSOR *) malloc(sizeof(ATTITUDE_SENSOR));
   
    /**
     * Try to open the hidraw device for reading the raw data. 
     * In future using libusb would be more elegant. 
     */
    self->fileDevice = open(
		ATTITUDE_SENSOR_HIDRAW,
		O_RDWR | O_NONBLOCK);

	if(self->fileDevice < 0) {
        LOG("Could not open device.");
	    return;
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
        fclose(file);
        attitude_sensor_read_config(self)
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
    for(int i = 0; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){
		self->ringbuffer_acc_pitch.measures[i] = 0.0;
		self->ringbuffer_acc_roll.measures[i] = 0.0;
	}

    /**
     * Everything went fine.
     */
	self->vuzix_connected = true;
  
    LOG("AttitudeSensor instantiated.");
    return self;
}

void attitude_sensor_delete(ATTITUDE_SENSOR *self) {
    free(self);
}

const HEAD_DIRECTION* attitude_sensor_get_head(ATTITUDE_SENSOR *self) {
	return self->head_direction;
}

void attitude_sensor_read_config(ATTITUDE_SENSOR *self) {

    LOG("Reading configuration");
    	
	string line;
	getline(configFile, line);
	this->biasGyro.x=atoi(line.c_str());
	getline(configFile, line);
	this->biasGyro.y=atoi(line.c_str());
	getline(configFile, line);
	this->biasGyro.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMin.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMin.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMin.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMax.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMax.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMax.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMin.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMin.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMin.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMax.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMax.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMax.z=atoi(line.c_str());
}

void attitude_sensor_write_config(ATTITUDE_SENSOR *self) {
	LOG("Writing configuration");
    
    ofstream configFileOut("attitudesensor.conf");
    configFileOut << this->biasGyro.x<<endl;
	configFileOut << this->biasGyro.y<<endl;
	configFileOut << this->biasGyro.z<<endl;
	configFileOut << this->calibMagMin.x<<endl;
	configFileOut << this->calibMagMin.y<<endl;
	configFileOut << this->calibMagMin.z<<endl;
	configFileOut << this->calibMagMax.x<<endl;
	configFileOut << this->calibMagMax.y<<endl;
	configFileOut << this->calibMagMax.z<<endl;
	configFileOut << this->calibAccMin.x<<endl;
	configFileOut << this->calibAccMin.y<<endl;
	configFileOut << this->calibAccMin.z<<endl;
	configFileOut << this->calibAccMax.x<<endl;
	configFileOut << this->calibAccMax.y<<endl;
	configFileOut << this->calibAccMax.z<<endl;
	configFileOut.close();

}

void attitude_sensor_receive(ATTITUDE_SENSOR *self) {
	bytesRead = read(
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
        current_angles.yaw, 
        current_acc_pitch, 
        current_acc_roll);

	ret_val.pitch = calculate_pitch(
        current_angles.pitch, 
		normalized_acc_sensor_data, 
		normalized_gyr_sensor_data, 
		current_gyro, 
		ringbuffer_acc_pitch, 
		current_acc_pitch);
	
	ret_val.roll = calculate_roll(
		current_angles.roll,
		normalized_acc_sensor_data, 
		normalized_gyr_sensor_data, 
		current_gyro, 
		ringbuffer_acc_roll, 
		current_acc_roll);
	
	return ret_val;
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

	//Filter Acc Data
	ringbuffer_acc_pitch.measures[ringbuffer_acc_pitch.pointer] = 
		atan2(
            normalized_acc_sensor_data.x, 
			sqrt(normalized_acc_sensor_data.z * normalized_acc_sensor_data.z + 
			normalized_acc_sensor_data.y * normalized_acc_sensor_data.y)
		) / M_PI; 

    float acc_avg = 0.0;

	for(unsigned int i = 0; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){
		
        unsigned int current_index =
			(ringbuffer_acc_pitch.pointer + i) % 
                ATTITUDE_SENSOR_RINGBUFFER_SIZE;
		
        float coefficient = geometric_distribution(
				ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY,
				i);

		acc_avg += coefficient * ringbuffer_acc_pitch.measures[current_index];	
	}

	ringbuffer_acc_pitch.pointer = ringbuffer_acc_pitch.pointer + 1;
	
    if(ringbuffer_acc_pitch.pointer == ATTITUDE_SENSOR_RINGBUFFER_SIZE){
		ringbuffer_acc_pitch.pointer = 0;	
	}
		
	//Gyro is difference, acc is absolute position
	acc_avg *= 90.0;
	current_acc_pitch = accAvg;

	//Add gyro to currentPitch
	current_pitch = 
        current_pitch + normalized_gyr_sensor_data.y * (90.0/32768.0);

	float possible_error_pitch = current_acc_pitch - current_pitch;

	if(possible_error_pitch > 2.0 || possible_error_pitch < -2.0){
		current_pitch = current_pitch + 0.05 * possible_error_pitch;
	}

	ret_val = current_pitch;
	return ret_val; 
}

float calculate_roll(
	float *current_roll,
	IWRSENSOR_PARSED_F *normalized_acc_sensor_data,
	IWRSENSOR_PARSED *normalized_gyr_sensor_data, 
	ANGLES *current_gyro,
	RINGBUFFER *ringbuffer_acc_roll,
	float *current_acc_roll){

	float ret_val;

    //Filter Acc Data
	ringbuffer_acc_roll.measures[ringbuffer_acc_roll.pointer]=
		atan2(	
			sqrt(normalized_acc_sensor_data.x * normalized_acc_sensor_data.x + 
			normalized_acc_sensor_data.z * normalized_acc_sensor_data.z),
			normalized_acc_sensor_data.y
		) / M_PI; 
	
    float acc_avg=0.0;
	
    for(unsigned int i=0; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){

		unsigned int current_index =
			(ringbuffer_acc_roll.pointer + i) % ATTITUDE_SENSOR_RINGBUFFER_SIZE;
	
        float coefficient = geometric_distribution(
				ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY,i);
		
        accAvg += coefficient * ringbuffer_acc_roll.measures[current_index];	
	}

	ringbuffer_acc_roll.pointer = ringbuffer_acc_roll.pointer + 1;
	
    if(ringbuffer_acc_roll.pointer == ATTITUDE_SENSOR_RINGBUFFER_SIZE){
		ringbuffer_acc_roll.pointer = 0;	
	}
		
	current_acc_roll = acc_avg * 90.0;

	//Add gyro to currentPitch
	current_roll = current_roll + normalized_gyr_sensor_data.z * 0.5 *(180.0/32768.0);
	float possible_error_roll = current_acc_roll - currentRoll;

	if(possible_errorRoll > 1.0) {
		current_roll = current_roll + 0.05 * possible_error_roll;
	} else if(possible_error_roll < -1.0) {
		current_roll = current_roll + 0.05 * possible_error_roll;
	}
	ret_val = current_roll;
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
    IWRSENSOR_PARSED_F mag = normalized_mag_sensor_data;
	float ret_val;
	float mag_yaw;
	
	float gyr_diff = normalized_gyr_sensor_data.x * 0.2 * (180.0 / 32768.0);

	double xh=mag.x;
	double yh=mag.y;
	double length = sqrt(xh * xh + yh * yh);
	xh = xh / length;
	yh = yh / length;
	magYaw = atan2(xh, yh); 
	magYaw*= 0.2 * (180.0 / 32768.0);
	
	float mag_diff = mag_yaw - last_mag_yaw;
	float possible_error_yaw = gyr_diff - mag_diff;

	if(possible_error_yaw > 1.0){
		ret_val = current_gyro.yaw + gyr_diff - 0.05 * possible_error_yaw;
	} else if(possible_error_yaw < -1.0){
		ret_val = current_gyro.yaw + gyr_diff + 0.05 * possible_error_yaw;
	} else {
		ret_val = current_gyro.yaw + gyr_diff;
	}

	current_gyro.yaw = ret_val;
	return ret_val;
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
	
	for(int i = 1; i < 250; i++) {//TODO define
		
        attitude_sensor_receive(self);

		for(int k = 0; k < 3; k++) {
			if(ptr_mag_min[k] > ptr[k]){
				ptr_mag_min[k] = ptr[k];
			} else if(ptr_mag_max[k] < ptr[k]){
				ptr_mag_max[k] = ptr[k];
			}
		}
		   
		for(int k = 3; k < 6; k++) {
				if(ptr_acc_min[k - 3] > ptr[k]) {
					ptr_acc_min[k - 3] = ptr[k];
				} else if(ptr_acc_max[k - 3] < ptr[k]) {
					ptr_acc_max[k - 3] = ptr[k];
				}
		}

		usleep(50000); //50 millis

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

    for(int i = 0; i < 4000; i++) {//TODO define
		attitude_sensor_receive(self);
		//int16_t **ptr = (int16_t **) &parsed; //TODO wieso nicht?
		int16_t *ptr = (int16_t *) &(self->parsed);
		for(int k = 6; k < 9; k++) {
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
	return ret_val;
}	

// -----------------

float normalizeValue(
    int16_t &min, 
    int16_t &max,
    int16_t &value){

    if(value < min) value = min;
	if(value > max) value = max;
	
	float retVal=
		2.0f*
		(((float)value - (float)min))/
		((float)max - (float)min)
		-1.0f;

	if(retVal<0.001f && retVal>-0.001f){
		std::cout<<"sozusagen 0"<<std::endl;
	}

	return retVal;
}

IWRSENSOR_PARSED normalize_gyro(
    IWRSENSOR_PARSED &biasGyro, 
	IWRSENSOR_PARSED &sensor){

	IWRSENSOR_PARSED retVal;	
	retVal.x = sensor.x - biasGyro.x;
	retVal.y = sensor.y - biasGyro.y;
	retVal.z = sensor.z - biasGyro.z;
	
    return retVal;
}

IWRSENSOR_PARSED_F normalize_sensor(
    IWRSENSOR_PARSED &calibMin,  
	IWRSENSOR_PARSED &calibMax, 
    IWRSENSOR_PARSED &sensor) {

	IWRSENSOR_PARSED_F retVal;		
	
    retVal.x=normalizeValue(calibMin.x,calibMax.x,sensor.x);
	retVal.y=normalizeValue(calibMin.y,calibMax.y,sensor.y);
	retVal.z=normalizeValue(calibMin.z,calibMax.z,sensor.z);

    return retVal;
}

IWRSENSDATA_PARSED parse_data() {

	IWRSENSDATA_PARSED ret;
	unsigned char *ptrData = (unsigned char*) &this->sensdata;
	signed short *ptrRet = (signed short*) &ret;

	for(int i = 0, j=0; i < 12; i+=2,  j++) { //mag and acc
		unsigned char *lsb = ptrData + i;
		unsigned char *msb = ptrData + i + 1;
		*(ptrRet + j) = (((unsigned short) *msb << 8) | (unsigned short) *lsb);
	} 

//	for(int i = 12, j=6; i < 18; i+=2,  j++) { //high bandwidth gyro
	for(int i = 18, j=6; i < 24; i+=2,  j++) { //low bandwidth gyro
		unsigned char *lsb = ptrData + i;
		unsigned char *msb = ptrData + i + 1;
		*(ptrRet + j) = (((unsigned short) *msb << 8) | (unsigned short) *lsb);
	} 
	return ret;
}

void timer_proc() {
	if(!this->vuzixConnected){
			return;
	}
	receive();

	double yaw, pitch, roll;
	
	IWRSENSOR_PARSED_F normalizedMagSensorData =
	    normalize_sensor(
        this->calibMagMin, 
	    this->calibMagMax, 
        this->parsed.mag_sensor);
	
	IWRSENSOR_PARSED_F normalizedAccSensorData = 
        normalize_sensorparse_data(
        this->calibAccMin, 
	    this->calibAccMax, 
        this->parsed.acc_sensor);

	IWRSENSOR_PARSED normalizedGyrSensorData = 
	    normalize_gyro(
        this->biasGyro, 
	    this->parsed.gyro_sensor);	
	
	ANGLES angles=calculate_angles(
	    this->currentAngles,		
	    normalizedMagSensorData, 
	    normalizedAccSensorData, 
	    normalizedGyrSensorData, 
	    this->currentGyro, 
	    this->ringbufferAccPitch, 
	    this->ringbufferAccRoll, 
	    this->currentAccPitch,
	    this->currentAccRoll);

	pitch = angles.pitch;
	roll = angles.roll;
	yaw = angles.yaw;
	
	if(this->useYaw)
		this->head->angles.yawDeg = angles.yaw - this->zeroAngles.yaw; 
	
	if(this->usePitch)
		this->head->angles.pitchDeg = angles.pitch- this->zeroAngles.pitch; 
	
	if(this->useRoll)
		this->head->angles.rollDeg = angles.roll -this->zeroAngles.roll;
}

void reset_head_direction() {
	this->zeroAngles.yaw=this->currentGyro.yaw;
	this->zeroAngles.pitch=this->currentAngles.pitch;
	this->zeroAngles.roll=this->currentAngles.roll;
	
	printf("Zeroes: %f || %f || %f\n", this->zeroAngles.yaw, 
	this->zeroAngles.pitch, this->zeroAngles.roll);
}

void toggle_use_yaw() {
	this->useYaw= (!this->useYaw);
}	

void toggle_use_pitch() {
	this->usePitch= (!this->usePitch);
}	

void toggle_use_roll() {
	this->useRoll= (!this->useRoll);
}	
