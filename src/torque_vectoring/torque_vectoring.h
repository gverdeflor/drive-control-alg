#pragma once

/*
 * torque_vectoring.h -- public interface to torque vectoring module
 */

#include <stdio.h>
#include <math.h>

typedef struct force_z {
    double force_z_rear_left;
    double force_z_rear_right;
    double force_z_front_left;
    double force_z_front_right;
} force_z_t;

typedef struct force_y {
    double force_y_front_right;
    double force_y_front_left;
    double force_y_rear_right;
    double force_y_rear_left;
} force_y_t;

typedef struct torque_max {
    double torque_max_rear_left;
    double torque_max_rear_right;
} torque_max_t;

typedef struct torque_requested {
    double torque_requested_rear_right;
    double torque_requested_rear_left;
} torque_requested_t;

typedef struct torque_corrected {
    double torque_corrected_rear_right;
    double torque_corrected_rear_left;
} torque_corrected_t;


/* 
 * calculates torque request based on throttle position and max torque
 */
double calcTorqueRequest(double throttlePosition, double velocityCG);

/* 
 * calculates turn radius based on steering angle 
 */
double calcTurnRadius(double steeringAngle);

/* 
 * calculates yaw rate request based on steering angle and velocity of center of gravity 
 */
double calcYawRateRequest(double steeringAngle, double velocityCG);

/* 
 * calculates yaw error based on difference between requested yaw rate and current yaw rate
 */
double calcYawError(double requestedYawRate, double currYawRate);

/* 
 * calculates yaw moment request based on yaw error, yaw rates, steering angle, and velocity of center of gravity  
 */
double calcYawMomentRequest(double yawError, double currYawRate, double requestedYawRate, double prevYawRate, double steeringAngle, double velocityCG, double timestep);

/* 
 * calculates torque distribution delta based on requested yaw moment
 */
double calcTorqueDistributionDelta(double requestedYawMoment);

/* 
 * calculates torque command based on steering angle, torque request, and torque distribution delta
 */
torque_requested_t calcRequestedTorque(double steeringAngle, double torqueRequest, double torqueDelta);

/* 
 * calculates vertical load weight transfer based on longitudinal and lateral acceleration
 */
force_z_t calcVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude);

/* 
 * calculates vertical load weight transfer based on longitudinal and lateral acceleration
 */
force_y_t calcLateralForces(double accelerationLatitude, force_z_t forces_z);

/* 
 *  calculates traction limit torque based on lateral acceleration and forces in Z-direction
 */
torque_max_t calcTractionLimitTorque(force_y_t forces_y, force_z_t forces_z);

/* 
 *  calculates traction limit torque based on requested torque and maximum torque
 */
torque_corrected_t checkTractionLimit(torque_requested_t requestedTorque, torque_max_t torqueMax);
