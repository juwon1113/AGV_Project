/*
 * agv_control.h
 *
 *  Created on: 2026. 5. 7.
 *      Author: juwon
 */

#ifndef INC_AGV_CONTROL_H_
#define INC_AGV_CONTROL_H_

typedef enum {
	CMD_FORWARD,			// 0x01
	CMD_STOP,				// 0x02
	CMD_LIFT_UP,			// 0x03
	CMD_LIFT_DOWN,			// 0x04
	CMD_ROTATE_LEFT,		// 0x05
	CMD_ROTATE_RIGHT,		// 0x06
	CMD_ROTATE_180			// 0x07
} Command;

// AGV 전진하는 함수
void agv_forward(int speed);

// AGV 정지하는 함수
void agv_stop(void);

// AGV 좌회전하는 함수
void agv_rotate_left(int speed);

// AGV 우회전하는 함수
void agv_rotate_right(int speed);

// AGV 180도 회전하는 함수
void agv_rotate_180(int speed);

#endif /* INC_AGV_CONTROL_H_ */
