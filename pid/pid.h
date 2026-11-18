/*
 * pid.h
 *
 *  Created on: 2026/11/17
 *      Author: ryoma
 */

#ifndef PID_PID_H_
#define PID_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float kp;
	float ki;
	float kd;
	float dt;
	float minTarget;
	float maxTarget;
	float minOut;
	float maxOut;
	float minIntegral;
	float maxIntegral;
	float minError;
	float maxError;
	float minDeadband;
	float maxDeadband;
}PIDConfig;

typedef struct {
	PIDConfig config;
	float target;
	float feedback;
	float error;
	float integral;
	float pOut;
	float iOut;
	float dOut;
	float out;
} PID;

void roboB_PID_init(PID *pid, PIDConfig config);
void roboB_PID_setConfig(PID *pid, PIDConfig config);
void roboB_PID_reset(PID *pid);
float roboB_PID_calc(PID *pid, float target, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* PID_PID_H_ */
