/*
 * pid.c
 *
 *  Created on: 2026. 3. 15.
 *      Author: juwon
 */

#include "main.h"
#include "pid.h"

// 내부 전용 함수 선언
static void PID_CalculateErrors(PID_t *pid, float current);

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt,
		float target, float outMax, float outMin)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->dt = dt;
	pid->target = target;
	pid->outMax = outMax;
	pid->outMin = outMin;

	if (Ki > 0)
	{
		pid->iMax = pid->outMax / Ki;
		pid->iMin = pid->outMin / Ki;
	}
	else
	{
		pid->iMax = 0;
		pid->iMin = 0;
	}

    pid->pTerm = 0.0f;
    pid->iTerm = 0.0f;
    pid->dTerm = 0.0f;
    pid->pidOutput = 0.0f;
    pid->error = 0.0f;
    pid->errorSum = 0.0f;
    pid->prevError = 0.0f;
}

void PID_SetTarget(PID_t *pid, float target)
{
	pid->target = target;
}

void PID_SetGain(PID_t *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	if (Ki > 0)
	{
		pid->iMax = pid->outMax / Ki;
		pid->iMin = pid->outMin / Ki;
	}
	else
	{
		pid->iMax = 0;
		pid->iMin = 0;
	}
}

void PID_Compute(PID_t *pid, float current)
{
	PID_CalculateErrors(pid, current);

	// pControl
	pid->pTerm = pid->Kp * pid->error;

	// iControl
	pid->iTerm = pid->Ki * pid->errorSum;

	// dControl
	pid->dTerm = pid->Kd * ((pid->error - pid->prevError) / pid->dt);

	pid->pidOutput = pid->pTerm + pid->iTerm + pid->dTerm;

	// 최종 출력 제한
	if (pid->pidOutput > pid->outMax) pid->pidOutput = pid->outMax;
	if (pid->pidOutput < pid->outMin) pid->pidOutput = pid->outMin;
}

// 내부 전용 함수 정의
static void PID_CalculateErrors(PID_t *pid, float current)
{
	pid->prevError = pid->error;
	pid->error = pid->target - current;
	pid->errorSum += pid->error * pid->dt;

	// Anti-windup: Ki 기준 Integral Clamping 적용
	if (pid->errorSum > pid->iMax) pid->errorSum = pid->iMax;
	if (pid->errorSum < pid->iMin) pid->errorSum = pid->iMin;
}
