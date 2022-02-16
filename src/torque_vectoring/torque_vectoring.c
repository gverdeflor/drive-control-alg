#include <stdio.h>
#include <math.h>
#include "torque_vectoring.h"

#define maxTorque   180         // Nm (Newton * meters)
#define l_r         .70         // m (meters)
#define l_f         1.05        // m (meters)
#define K_u         -.06902     // K constant
#define t_f         .75         // m (meters)
#define F_yfl       7           // UPDATE
#define F_yfr       7           // UPDATE
#define t_r         .7          // m (meters)
#define F_xrr       7           // m (meters)
#define F_xrl       7           // m (meters)
#define r           .28575      // cm
#define mu          1.7         // constant
#define G           5.6         // constant
#define L           1.75        // m (meters)


int helper_func(int returnval) {
    return returnval;
}

/*
 * Function: calculateTorqueRequest
 * --------------------------------
 * calculates the torque request based on throttle position and max torque
 * 
 * throttlePosition: pedal compression % (from throttle position sensor)
 * 
 * returns: the requested torque as a percent of the maximum available torque
 */
double calculateTorqueRequest(double throttlePosition) {
    double requestedTorque = maxTorque * throttlePosition;
    return requestedTorque;
}


/*
 * Function: calculateDesiredYawRate
 * ---------------------------------
 * calculates the yaw rate request based on steering angle and velocity of the center of gravity
 * 
 * steeringAngle: from steering angle sensor
 * velocityCG: GPS Speed - from vehicle dynamics module (VDM)
 * returns: desired yaw rate
 */
double calculateDesiredYawRate(double steeringAngle, double velocityCG) {
    double desiredYawRate = velocityCG * steeringAngle / ((l_r + l_f) + K_u * pow(velocityCG, 2));
    return desiredYawRate;
}

/*
 * Function: calculateYawError
 * ---------------------------
 * calculates the yaw error based on the difference between the desired rate and the current rate
 * 
 * desiredYawRate: desired yaw rate based on driver input (steering angle sensor + vehicle speed)
 * currentYawRate: current yaw rate of the vehicle - from vehicle dynamics module (VDM)
 * returns: difference between the desired rate and the current rate
 */
double calculateYawError(double desiredYawRate, double currentYawRate) {
    double yawError = desiredYawRate - currentYawRate;
    return yawError;
}


/*
 * Function: calculateDesiredYawMoment
 * ---------------------------
 * calculates the yaw error based on the difference between the desired rate and the current rate
 * 
 * desiredYawRate: desired yaw rate based on driver input (steering angle sensor + vehicle speed)
 * currentYawRate: current yaw rate of the vehicle - from vehicle dynamics module (VDM)
 * returns: difference between the desired rate and the current rate
 */
double calculateDesiredYawMoment(double yawError, double steeringAngle) {
    double desiredYawMoment = t_f * sin(steeringAngle) * (F_yfl - F_yfr) + t_r * (F_xrr - F_xrl);
    return desiredYawMoment;
}



// double calculateLateralForces(double yaw_rate, double velocityLongitude, double velocityLatitude, double steeringAngle) {
//     F_yf = - C_f * 
// }

ForceZStruct calculateVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude) {
    // needs to be filled in
    ForceZStruct forces;
    forces.F_zrl = 1;
    forces.F_zrr = 1;

    return forces;
}


TMaxStruct calculateTractionLimitTorque(double F_yrl, double F_yrr, double F_zrl, double F_zrr) {
    TMaxStruct tmaxes;
    double TMax_rl = r * sqrt(pow((mu * F_zrl), 2.0) - F_yrl);
    double TMax_rr = r * sqrt(pow((mu * F_zrr), 2.0) - F_yrr);
    
    tmaxes.TMax_rl = TMax_rl;
    tmaxes.TMax_rr = TMax_rr;

    return tmaxes;
}

double calculateTorqueDistributionDelta(double desiredYawMoment, double steeringAngle, double velocityCG) {
    double cot = 1; // REMOVE
    double w = 1; // REMOVE

    double R = sqrt(pow((t_f / 2), 2.0) + pow(L, 2.0) * pow(cot, 2.0) * steeringAngle);
    // double R = (I + K_u * velocityCG) / steeringAngle;
    double torqueDistributionDelta = desiredYawMoment * R * w / (t_r * G);
    return torqueDistributionDelta;
}