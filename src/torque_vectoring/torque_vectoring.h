#pragma once

/*
 * torque_vectoring.h -- public interface to torque vectoring module
 */

#include <stdio.h>
#include <math.h>

/* 
 * calculates torque request based on throttle position and max torque
 */
double calcTorqueRequest(double throttlePosition);

/* 
 * calculates yaw rate request based on steering angle and velocity of center of gravity 
 */
double calcYawRateRequest(double steeringAngle, double velocityCG);

/* 
 * calculates yaw error based on difference between requested yaw rate and current yaw rate
 */
double calcYawError(double desiredYawRate, double currYawRate);