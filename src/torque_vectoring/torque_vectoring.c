#include <stdio.h>
#include <math.h>

#define maxTorque   180 // Nm (Newton * meters)
#define l_r         .70 // m (meters)
#define l_f         1.05 // m (meters)
#define K_u         -.06902 // K constant


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