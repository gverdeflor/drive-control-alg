/* torque_vectoring.c ---
 * 
 *
 * Author: Suraj Srivats, Garth Verdeflor
 * Created: Mon Feb 14 11:33:29 2022 (-0400)
 * Version: 1.0  
 *
 *
 * Description: Implements torque vectoring algorithm.
 *
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define maxTorque   180         // Nm
#define lFront      0.70        // m
#define lRear       1.05        // m
#define K_u         -0.06902    // K constant - WHAT IS THIS?

//------------------------ calcTorqueRequest -----------------------
// Description:   calculates torque request 
// Inputs:        throttle position and max torque
// Outputs:       requested torque as % of maximum torque available
//------------------------------------------------------------------
double calcTorqueRequest(double throttlePosition) {
    double requestedTorque = maxTorque * throttlePosition;
    return requestedTorque;
}

//-------------------------- calcYawRate ---------------------------
// Description:   calculates yaw rate request
// Inputs:        steering angle and velocity of center of gravity
// Outputs:       desired yaw rate
//------------------------------------------------------------------
double calcYawRateRequest(double steeringAngle, double velocityCG) {
    double requestedYawRate = (velocityCG * steeringAngle / (lFront + lRear)) + (K_u * pow(velocityCG, 2));
    return requestedYawRate;
}

//------------------------- calcYawError ---------------------------
// Description:   calculates yaw error
// Inputs:        requested yaw rate and current yaw rate
// Outputs:       yaw error (i.e., difference between requested and current rate)
//------------------------------------------------------------------
double calcYawError(double requestedYawRate, double currYawRate) {
    double yawError = requestedYawRate - currYawRate
    return yawError;
}