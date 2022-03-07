#pragma once

/*
 * torque_vectoring.h -- public interface to torque vectoring module
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

typedef struct forceZ {
    double F_zrl;
    double F_zrr;
    double F_zfl;
    double F_zfr;
} force_z_t;

typedef struct forceY {
    double F_yfr;
    double F_yfl;
    double F_yrr;
    double F_yrl;
} force_y_t;

typedef struct torqueMax {
    double maxTorqueRL;
    double maxTorqueRR;
} torque_max_t;

typedef struct torqueDesired {
    double desiredTorqueRR;
    double desiredTorqueRL;
} torque_desired_t;

typedef struct torqueNew {
    double newTorqueRR;
    double newTorqueRL;
} torque_new_t;


/* 
 * calculates torque request based on throttle position and max torque
 */
double calcTorqueRequest(double throttlePosition, double velocityCG, bool endurance_bool);

/* 
 * calculates turn radius based on steering angle 
 */
double calcTurnRadius(double steeringAngle);

/* 
 * calculates yaw rate request based on steering angle and velocity of center of gravity 
 */
double calcDesiredYawRate(double steeringAngle, double velocityCG);

/* 
 * calculates yaw error based on difference between requested yaw rate and current yaw rate
 */
double calcYawError(double desiredYawRate, double currYawRate);

/* 
 * calculates yaw moment request based on yaw error, yaw rates, steering angle, and velocity of center of gravity  
 */
double calcYawMomentRequest(double yawError, double currYawRate, double requestedYawRate, double prevYawRate, double steeringAngle, double velocityCG, double timestep, force_z_t F);

/* 
 * calculates torque distribution delta based on requested yaw moment
 */
double calcTorqueDistributionDelta(double desiredYawMoment);

/* 
 * calculates torque command based on steering angle, torque request, and torque distribution delta
 */
torque_desired_t calcDesiredTorque(double steeringAngle, double torqueRequest, double torqueDelta);

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
torque_max_t calcTractionLimitTorque(force_y_t forcesY, force_z_t forcesZ);

/* 
 *  calculates traction limit torque based on requested torque and maximum torque
 */
torque_corrected_t checkTractionLimit(torque_requested_t requestedTorque, torque_max_t torqueMax, double torqueLimitBatteryState);
