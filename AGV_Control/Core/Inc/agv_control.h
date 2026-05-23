/*
 * agv_control.h
 *
 *  Created on: 2026. 5. 9.
 *      Author: juwon
 */

#ifndef INC_AGV_CONTROL_H_
#define INC_AGV_CONTROL_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "pid.h"
#include "BNO055_STM32.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

#define CMD_FORWARD       0x01
#define CMD_STOP          0x02
#define CMD_LIFT_UP       0x03
#define CMD_LIFT_DOWN     0x04
#define CMD_ROTATE_LEFT   0x05
#define CMD_ROTATE_RIGHT  0x06
#define CMD_ROTATE_180    0x07
#define CMD_CORRECTION	  0x08
#define ABS(x) (((x) > 0) ? (x) : (-x))

// 캘리브레이션 값이 저장되어있는 플래시 메모리 주소 (Sector11)
#define CALIB_ADDR  ((uint32_t)0x081C0000)

#define MOTOR_CPR			64.0f
#define GEAR_RATIO			70.0f
#define ENCODER_CPR			(MOTOR_CPR * GEAR_RATIO)
#define WHEEL_DIAMETER_MM	66.0f
#define WHEEL_CIRCUMFERENCE	(M_PI * WHEEL_DIAMETER_MM)
#define TARGET_DISTANCE		500.0f

#define ArUcoMarker_RxBuf_SIZE		19

void Set_Zero_Yaw(void);

void AGV_START(void);

void ENCODER_COUNT_UPDATE(void);

void SPEED_PID_Control(float target);

void DISTANCE_PID_Control(float target);

float ReadYawData(void);

void FORWARD_YAW_PID_Control(void);

void ROTATE_YAW_PID_Control(void);

int ArUcoMarker_Data_Parsing(const uint8_t* buf);

void ReadArUcoMarkerData(void);

void AGV_FORWARD(uint16_t basePwm, float target);

void AGV_STOP(void);

void AGV_ROTATE(uint16_t basePwm, float target);

void AGV_CORRECTION(uint16_t basePwm, float target);

#endif /* INC_AGV_CONTROL_H_ */
