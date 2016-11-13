// pid.h
// 12 October 2016 jowong@g.hmc.edu
// library for pid functions

#ifndef _INCLUDE_PID_
#define _INCLUDE_PID_

typedef struct {
	double target;
	double proportionalGain;
	double integralGain;
	double derivativeGain;
	double previousError;
	double integralError;
	double control;
} PID;

void pidInit(PID * pid) {
	pid->previousError = 0;
	pid->integralError = 0;
}

void setGain(PID * pid, double tgt, double pGain, double iGain, double dGain) {
	pid->target = tgt;
	pid->proportionalGain = pGain;
	pid->integralGain = iGain;
	pid->derivativeGain = dGain;
}

void pid_update(PID * pid, double data, double loopPeriod) {
	double currentError = data - pid->target;
	double pTerm = pid->proportionalGain * currentError;
	pid->integralError += currentError;
	double iTerm = pid->integralGain * integralError;
	double derivative = (currentError - pid->previousError) / loopPeriod;
	double dTerm = pid->derivativeGain * derivative;
	pid->control = pTerm + iTerm + dTerm;
}

#endif
