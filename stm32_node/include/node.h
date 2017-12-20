#ifndef _node_h
#define _node_h

#include <ros/ros.h>
#include <stdint.h>
#include <unistd.h>

// Package/protocol definitions
#define CRC16 // using CRC16 for checksum
#define PACKAGE_LENGTH	17
#define HEADER_BYTE	0xFE

#define CMD_ACTUATOR	0x01
#define CMD_BUZZER_BEEP	0x0A
#define CMD_LED 	0x0B
#define CMD_BOOTLOADER	0x0F

#define CMD_LIDAR	0x10
#define CMD_ENCODER	0x11
#define CMD_RC  	0x12

typedef struct LiDAR_Measurement {
	ros::Time Timestamp;
	uint16_t Distance;
} LiDAR_Measurement;

int main(int argc, char** argv);
int ParsePackage(uint8_t * package);
int ParseLiDARMeasurement(uint8_t * package);
int ParseEncoderMeasurement(uint8_t * package);
int ParseRCMeasurement(uint8_t * package);

#endif /* _node_h */
