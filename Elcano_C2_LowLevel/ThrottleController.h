#pragma once
#include "Settings.h"
#ifndef TESTING
#include <SPI.h>
#include <MCP48x2.h>
#include "PID_v1.h" 
#endif

#define MAX_VALUE 1023
#define CHANNEL_A 0x00
#define CHANNEL_B 0x80
#define GAIN_1 0x20
#define GAIN_2 0x00
#define MODE_SHUTDOWN 0x00
#define MODE_ACTIVE 0x10

class ThrottleController{
private:
	volatile int32_t currentThrottlePWM;
	double speedCyclometerInput_mmPs = 0;
	double PIDThrottleOutput_pwm;
	double desiredSpeed_mmPs = 0;
  //MCP48x2 dac(MCP4802, DAC_SS);

	PID speedPID;

	static const uint32_t MIN_TICK_TIME_ms = (WHEEL_CIRCUM_MM * 1000) / MAX_SPEED_mmPs;
	static const uint32_t MAX_TICK_TIME_ms = (WHEEL_CIRCUM_MM * 1000) / MIN_SPEED_mmPs;
	
	static volatile uint32_t tickTime_ms[2];
	uint32_t calcTime_ms[2];
	int32_t prevSpeed_mmPs;
	
	void write(int32_t address, int32_t value);
	void ThrottlePID(int32_t desiredValue);
	int32_t extrapolateSpeed();
	void computeSpeed(); 
	void engageThrottle(int32_t input);	
	static void tick();
public:
	
	ThrottleController();
	~ThrottleController();
	void stop();
	int32_t update(int32_t dSpeed);

};
