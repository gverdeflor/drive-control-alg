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
#include "torque_vectoring.h"

#define maxTorque   178                                                     // Nm (89 Nm for EMRAX 188 MV motor)
#define carWeight   0.6                                                     // % weight in the rear (estimated)
#define L           1.75                                                    // m - wheel base to wheel base length
#define l_r         (1 - carWeight) * L                                     // m - length from CG to rear
#define l_f         (carWeight) * L                                         // m - length from CG to front
#define mass        360                                                     // 360 kg - estimated from previous car
#define c_f         529                                                     // N/deg - cornerning stiffness (front) - estimated from similar SAE car --> needs to be converted to radians
#define c_r         633                                                     // N/deg - cornering stiffness (rear) - estimated from similar SAE car --> needs to be converted to radians
#define K_u         (l_r * mass / (c_f * L)) - (l_f * mass / (c_r * L))     // K constant
#define t_f         1.35                                                    // m (meters) - distance between two front wheels
#define t_r         1.3                                                     // m (meters) - distance between two rear wheels

#define r           .28575                                                  // m - wheel radius
#define mu          1.7                                                     // constant - coefficient of friction (for now)
#define G           5.6                                                     // constant - gear ratio between motor and wheel
#define heightCG    .27                                                     // m - height of center of gravity (off the ground)
#define h_s         .28                                                     // roll (?)
#define K_r         570.0197                                                // roll stiffness rear
#define K_f         546.4094                                                // roll stiffness front
#define p_r         h_s * (K_r / (K_f + K_r))                               // roll distribution rear (units?)
#define p_f         h_s * (K_f / (K_f + K_r))                               // roll distribution rear (units?)
#define I_z         120                                                     // kg * (m) ^ 2 - moment of inertia in the z axis
#define grav        9.82                                                    // m / (s) ^ 2    // gravity


/*
 * Function: calcTorqueRequest
 * --------------------------------
 * calculates torque request based on throttle position and max torque,
 * in the case that throttlePosition = 0, it does regen braking
 * 
 *      throttlePosition: pedal compression [0 - 100] (from throttle position sensor)
 *      velocityCG: GPS speed [m/s] (from vehicle dynamics module)
 * 
 * returns: requested torque as % of the maximum available torque
 */
double calcTorqueRequest(double throttlePosition, double velocityCG, bool endurance_bool) {
    double requestedTorque;
    requestedTorque = maxTorque * (throttlePosition / 100);

    if (requestedTorque == 0) {
        if (endurance_bool) {
            double targetA = 2;

            double maxP = 15.2384;
            double dragCoeff = .6;
            double airDensityCoeff = 1.225;
            double frontalAreaOfCar = .4;
            double dragForce = ((.5 * dragCoeff * airDensityCoeff * frontalAreaOfCar) * pow(velocityCG, 2));
            double rollingForce = 88.24;
            double motorRPM = 60 * velocityCG / (2 * M_PI * r) * G;
            double rpmCoversion = 9.5492965964254;
            double motorRadSec = motorRPM / rpmCoversion;

            double forceDueToMotor = (1 - exp(- velocityCG / .8)) * ((targetA * mass) - dragForce - rollingForce);
            double torqueAtWheel = r * forceDueToMotor;

            double torqueAtMotor;
            if (((torqueAtWheel / (G * .94)) * motorRadSec) < (maxP * 1000)) {
                torqueAtMotor = torqueAtWheel / (G * .94);
            } else {
                torqueAtMotor = 1000 * maxP / motorRadSec;
            }

            requestedTorque = - torqueAtMotor;
        } else {
            requestedTorque = -10;
        }
    }
    return requestedTorque;
}

/*
 * Function: calcTurnRadius
 * -----------------------------
 * calculates the turn radius based on the steering angle
 * 
 *      steeringAngle: from steering angle sensor
 *
 * returns: the calculated turn radius
 */
double calcTurnRadius(double steeringAngle) {
    double turnRadius;
    if (steeringAngle != 0) {
        turnRadius = sqrt(pow(t_f/4, 2) + (pow(L, 2) * pow(1 / tan(steeringAngle), 2)));
    } else {
        turnRadius = sqrt(pow(t_f/4, 2) + (pow(L, 2) * pow(1 / tan(steeringAngle + .000001), 2)));
    }

    return turnRadius;
}

/*
 * Function: calcYawRateRequest
 * ---------------------------------
 * calculates the yaw rate request based on steering angle and velocity of the center of gravity
 * 
 *      steeringAngle: from steering angle sensor
 *      velocityCG: GPS speed [m/s] (from vehicle dynamics module)
 *
 * returns: desired yaw rate
 */
double calcYawRateRequest(double steeringAngle, double velocityCG) {
    double desiredYawRate;
    double turnRadius = calcTurnRadius(steeringAngle);

    if (steeringAngle > 0) {
        desiredYawRate = (velocityCG / turnRadius);
    } else {
        desiredYawRate = - (velocityCG / turnRadius);
    }

    return desiredYawRate;
}

/*
 * Function: calcYawError
 * ---------------------------
 * calculates the yaw error based on the difference between the desired rate and the current rate
 * 
 *      desiredYawRate: desired yaw rate based on driver input (steering angle sensor + vehicle speed)
 *      currentYawRate: current yaw rate of the vehicle (from vehicle dynamics module)
 *
 * returns: difference between desired yaw rate and current yaw rate
 */
double calcYawError(double desiredYawRate, double currentYawRate) {
    double yawError = desiredYawRate - currentYawRate;
    return yawError;
}


/*
 * Function: calcYawMomentRequest
 * -----------------------------------
 * calculates the yaw moment request based on the yaw error, yaw rates, steering angle, and velocity of center of gravity 
 * 
 *      yawError: calculated from above 
 *      currentYawRate: current yaw rate of the vehicle (from vehicle dynamics module)
 *      requestedYawRate: desired yaw rate based on driver input (steering angle sensor + vehicle speed)
 *      prevYawRate: previously calculated yaw rate
 *      steeringAngle: from steering angle sensor
 *      velocityCG: GPS speed [m/s] (from vehicle dynamics module)
 *      timstep: time between algorithm calculations
 *
 * returns: difference between the desired rate and the current rate
 */
double calcYawMomentRequest(double yawError, double currentYawRate, double requestedYawRate, double prevYawRate, double steeringAngle, double velocityCG, double timestep, force_z_t F) {
    double sideSlipAngleCoeff = c_f * l_f - c_r * l_r;
    double yawRateOverVxCoeff = c_f * pow(l_f, 2) + c_r * pow(l_f, 2);
    double steeringAngleCoeff = c_f * l_f;
    double eta = 2;

    double turnRadius = calcTurnRadius(steeringAngle);
    double signofSteeringAngle = (steeringAngle > 0) - (steeringAngle < 0);
    double beta = signofSteeringAngle * (57.3 * l_r / turnRadius - (F.force_z_rear_right + F.force_z_rear_left) * pow(velocityCG, 2) / (c_r * grav * turnRadius));
    
    double requestedYawRateDerivative = .174;

    double derivTerm = I_z * requestedYawRateDerivative;
    double sideslipTerm = sideSlipAngleCoeff * beta / 57.3; // beta in radians
    double yawOverVxTerm = yawRateOverVxCoeff * currentYawRate / velocityCG;
    double steeringAngleTerm = (steeringAngleCoeff / 57.3) * steeringAngle; // radians
    double yawErrorTerm = eta * I_z * yawError;

    double desiredYawMoment = derivTerm + sideslipTerm + yawOverVxTerm - steeringAngleTerm - yawErrorTerm;

    return desiredYawMoment;
}

/*
 * Function: calcTorqueDistributionDelta
 * ------------------------------------------
 * calculates the torque delta based off of the desired yaw moment
 * 
 *      desiredYawMoment: from the calculateDesiredYawMoment function (double)
 * 
 * returns: torque delta
 */
double calcTorqueDistributionDelta(double desiredYawMoment) {
    double torqueDelta = desiredYawMoment * r / (t_r * G);

    return torqueDelta;
}

/*
 * Function: calcRequestedTorque
 * --------------------------------
 * calculates the desired torque based off of the torque request, steering angle, and the torque delta
 * 
 *      steeringAngle: from steering angle sensor
 *      torqueRequest: calculated by torque request function
 *      torqueDelta: calculated by torque distribution delta function
 * 
 * returns: desired torque
 */
torque_requested_t calcRequestedTorque(double steeringAngle, double torqueRequest, double torqueDelta) {
    torque_requested_t requestedTorque;
    double requestedTorqueRR;
    double requestedTorqueRL;

    if (steeringAngle < 0) {
        requestedTorqueRR = 0.5 * (torqueRequest + torqueDelta);
        requestedTorqueRL = 0.5 * (torqueRequest - torqueDelta);
    } else {
        requestedTorqueRR = 0.5 * (torqueRequest - torqueDelta);
        requestedTorqueRL = 0.5 * (torqueRequest + torqueDelta);
    }

    requestedTorque.torque_requested_rear_right = requestedTorqueRR;
    requestedTorque.torque_requested_rear_left = requestedTorqueRL;

    return requestedTorque;
}

/*
 * Function: calcVerticalLoadWeightTransfer
 * ---------------------------------------------
 * calculates the weight transfer of the vertical load
 * 
 *      accelerationLongitude: from vehicle dynamics module
 *      accelerationLatitude: from vehicle dynamics module
 * 
 * returns: struct with the four z forces
 */
force_z_t calcVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude) {
    force_z_t F;
    F.force_z_rear_left = (0.5 * mass * grav * l_f / L) - (p_r * accelerationLatitude * mass * heightCG / t_r) + (.5 * accelerationLongitude * mass * heightCG / L);
    F.force_z_rear_right = (0.5 * mass * grav * l_f / L) + (p_r * accelerationLatitude * mass * heightCG / t_r) + (.5 * accelerationLongitude * mass * heightCG / L);

    F.force_z_front_left = (0.5 * mass * grav * l_r / L) - (p_f * accelerationLatitude * mass * heightCG / t_r) - (.5 * accelerationLongitude * mass * heightCG / L);
    F.force_z_front_right = (0.5 * mass * grav * l_r / L) + (p_f * accelerationLatitude * mass * heightCG / t_r) - (.5 * accelerationLongitude * mass * heightCG / L);

    return F;
}

/*
 * Function: calcLateralForces
 * --------------------------------
 * calculates the lateral forces
 * 
 *      accelerationLatitude: from the vehicle dynamics module
 *      Z: from calculateVerticalLoadWeightTransfer method
 * 
 * returns: struct with the four y forces
 */
force_y_t calcLateralForces(double accelerationLatitude, force_z_t Z) {
    force_y_t Y;
    Y.force_y_front_right = (Z.force_z_front_right / grav) * (-accelerationLatitude);
    Y.force_y_front_left = (Z.force_z_front_left / grav) * (-accelerationLatitude);
    Y.force_y_rear_right = (Z.force_z_rear_right / grav) * (-accelerationLatitude);
    Y.force_y_rear_left = (Z.force_z_rear_left / grav) * (-accelerationLatitude);

    return Y;
}

/*
 * Function: calcTractionLimitTorque
 * --------------------------------------
 * calculates the traction limit torque
 * 
 *      Y: from the calculate lateral forces method
 *      Z: from the calculate vertical load weight transfer method
 * 
 * returns: struct with the max traction limit torques
 */
torque_max_t calcTractionLimitTorque(force_y_t Y, force_z_t Z) {
    torque_max_t T;
    T.torque_max_rear_left = (r * sqrt(pow(mu * Z.force_z_rear_left, 2.0) - pow(Y.force_y_rear_left, 2.0))) / G;
    T.torque_max_rear_right = (r * sqrt(pow(mu * Z.force_z_rear_right, 2.0) - pow(Y.force_y_rear_right, 2.0))) / G;

    return T;
}

/*
 * Function: checkTractionLimit
 * --------------------------------------
 * calculates the traction limit torque
 * 
 *      R: requested torque struct from calculated desired torque from 
 *      M: max traction limit torques struct from traction limit torque method
 * 
 * returns: struct with the new torques
 */
torque_corrected_t checkTractionLimit(torque_requested_t R, torque_max_t M, double torqueLimitBatteryState) {
    torque_corrected_t C;
    double newTorqueRR;
    double newTorqueRL;

    if (R.torque_requested_rear_right >= 0 && R.torque_requested_rear_left >= 0) {

        if ((.9 * M.torque_max_rear_right > R.torque_requested_rear_right) && (.9 * M.torque_max_rear_left > R.torque_requested_rear_left)) {
            newTorqueRR = R.torque_requested_rear_right;
            newTorqueRL = R.torque_requested_rear_left;
        } else {
            double Tdiff = fmax(R.torque_requested_rear_right - .9 * M.torque_max_rear_right, R.torque_requested_rear_left -.9 * M.torque_max_rear_left);
            newTorqueRR = fmax(-.9 * M.torque_max_rear_right, R.torque_requested_rear_right - Tdiff);
            newTorqueRL = fmax(-.9 * M.torque_max_rear_left, R.torque_requested_rear_left - Tdiff);
        }
    } else if (R.torque_requested_rear_right >= 0 && R.torque_requested_rear_left < 0) {
        if(M.torque_max_rear_right > R.torque_requested_rear_right) {
            if(M.torque_max_rear_left > -R.torque_requested_rear_left) {
                newTorqueRR = R.torque_requested_rear_right;
                newTorqueRL = R.torque_requested_rear_left;
            } else {
                double Tdiff = -R.torque_requested_rear_left - M.torque_max_rear_left;
                newTorqueRR = R.torque_requested_rear_right - Tdiff;
                newTorqueRL = R.torque_requested_rear_left + Tdiff;
            }
        } else {
            double Tdiff = fmax(R.torque_requested_rear_right - M.torque_max_rear_right, -R.torque_requested_rear_left - M.torque_max_rear_left);
            newTorqueRR = R.torque_requested_rear_right - Tdiff;
            newTorqueRL = fmax(R.torque_requested_rear_left - Tdiff, -M.torque_max_rear_left);
        }
        
        if (-newTorqueRR > M.torque_max_rear_right) {
            double diff = -newTorqueRR - M.torque_max_rear_right;
            newTorqueRR = -M.torque_max_rear_right;
            newTorqueRL = fmax(newTorqueRL - diff, -M.torque_max_rear_left);
        }
    } else if (R.torque_requested_rear_right < 0 && R.torque_requested_rear_left >= 0) {
        if (M.torque_max_rear_left > R.torque_requested_rear_left) {
            if (M.torque_max_rear_right > -R.torque_requested_rear_right) {
                newTorqueRR = R.torque_requested_rear_right;
                newTorqueRL = R.torque_requested_rear_left;
            } else {
                double Tdiff = -R.torque_requested_rear_right - M.torque_max_rear_right;
                newTorqueRR = R.torque_requested_rear_right + Tdiff;
                newTorqueRL = R.torque_requested_rear_left - Tdiff;
            }
        } else {
            double Tdiff = fmax(R.torque_requested_rear_left - M.torque_max_rear_left, -R.torque_requested_rear_right - M.torque_max_rear_right);
            newTorqueRL = R.torque_requested_rear_left - Tdiff;
            newTorqueRR = fmax(R.torque_requested_rear_right - Tdiff, -M.torque_max_rear_right);
        }
        
        if (-newTorqueRL > M.torque_max_rear_left) {
            double diff = -newTorqueRL - M.torque_max_rear_left;
            newTorqueRL = -M.torque_max_rear_left;
            newTorqueRR = fmax(newTorqueRR - diff, -M.torque_max_rear_right);
        }
    } else if (R.torque_requested_rear_right < 0 && R.torque_requested_rear_left < 0) {
        if ((-.9* M.torque_max_rear_left < R.torque_requested_rear_left) && (-.9*M.torque_max_rear_right<R.torque_requested_rear_right)) {
            newTorqueRR = R.torque_requested_rear_right;
            newTorqueRL = R.torque_requested_rear_left;
        } else {
            double Tdiff = fmax(-R.torque_requested_rear_left - .9 * M.torque_max_rear_left, -R.torque_requested_rear_right < M.torque_max_rear_right);
            newTorqueRR = fmin(0, R.torque_requested_rear_right + Tdiff);
            newTorqueRL = fmin(0, R.torque_requested_rear_left + Tdiff);
        }

    }

    if (newTorqueRR > maxTorque / 2) {
        if (newTorqueRL >= 0) {
            double diff = fmax(newTorqueRR - maxTorque / 2, newTorqueRL - maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.torque_max_rear_right), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.torque_max_rear_left), -maxTorque / 2);
        } else {
            double diff = fmax(newTorqueRR - maxTorque / 2, -newTorqueRL - maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.torque_max_rear_right), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.torque_max_rear_left), -maxTorque / 2);
        }
    } else if (newTorqueRL > maxTorque / 2) {
        if (newTorqueRR >= 0) {
            double diff = newTorqueRL - maxTorque / 2;
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.torque_max_rear_left), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.torque_max_rear_right), -maxTorque / 2);
        } else {
            double diff = fmax(newTorqueRR - maxTorque / 2, -newTorqueRL - maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.torque_max_rear_left), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.torque_max_rear_right), -maxTorque / 2);
        }
    }

    if (-newTorqueRR > maxTorque / 2) {
        if (newTorqueRL <= 0) {
            double diff = fmax(-newTorqueRR - maxTorque / 2, -newTorqueRL -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR + diff, -M.torque_max_rear_right), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL + diff, -M.torque_max_rear_left), -maxTorque / 2);
        } else {
            double diff = fmax(-newTorqueRR - maxTorque / 2, newTorqueRL -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR + diff, -M.torque_max_rear_right), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.torque_max_rear_left), -maxTorque / 2);
        }
    } else if (-newTorqueRL > maxTorque / 2) {
        if (-newTorqueRR >= 0) {
            double diff = -newTorqueRL - maxTorque / 2;
            newTorqueRL = fmax(fmax(newTorqueRL + diff, -M.torque_max_rear_left), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR + diff, -M.torque_max_rear_right), -maxTorque / 2);
        } else {
            double diff = fmax(newTorqueRR - maxTorque / 2, -newTorqueRL - maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL + diff, -M.torque_max_rear_left), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.torque_max_rear_right), -maxTorque / 2);
        }
    }

    if (torqueLimitBatteryState < newTorqueRR + newTorqueRL) {
        double diff = torqueLimitBatteryState - (newTorqueRR + newTorqueRL);
        newTorqueRL = newTorqueRL - .5 * diff;
        newTorqueRR = newTorqueRR - .5 * diff;
    }

    C.torque_corrected_rear_left = newTorqueRL;
    C.torque_corrected_rear_right = newTorqueRR;

    return C;
}