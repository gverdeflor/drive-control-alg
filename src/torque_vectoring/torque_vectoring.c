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
#include "torque_vectoring.h"

#define maxTorque   178                                                     // Nm (89 Nm for EMRAX 188 MV motor)
#define weightDist   0.6                                                    // % weight in the rear (estimated)
#define L           1.75                                                    // m - wheel base to wheel base length
#define l_r         (1 - weightDist) * L                                    // m - length from CG to rear
#define l_f         (weightDist) * L                                        // m - length from CG to front
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
double calcTorqueRequest(double throttlePosition, double velocityCG) {
    double requestedTorque;
    requestedTorque = maxTorque * (throttlePosition / 100);

    if (requestedTorque == 0) {
        double targetA = 2;

        double maxP = 15.2384;
        double dragCoeff = 0.6;
        double airDensityCoeff = 1.225;
        double frontalAreaOfCar = 0.4;
        double dragForce = ((0.5 * dragCoeff * airDensityCoeff * frontalAreaOfCar) * pow(velocityCG, 2));
        double rollingForce = 88.24;
        double motorRPM = 60 * velocityCG / (2 * M_PI * r) * G;
        double rpmCoversion = 9.5492965964254;
        double motorRadSec = motorRPM / rpmCoversion;

        double forceDueToMotor = (1 - exp(- velocityCG / .8)) * ((targetA * mass) - dragForce - rollingForce);
        double torqueAtWheel = r * forceDueToMotor;

        double torqueAtMotor;
        if (((torqueAtWheel / (G * 0.94)) * motorRadSec) < (maxP * 1000)) {
            torqueAtMotor = torqueAtWheel / G / 0.94;
        } else {
            torqueAtMotor = 1000 * maxP / motorRadSec;
        }

        requestedTorque = - torqueAtMotor;
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
        turnRadius = sqrt(pow(t_f/4, 2) + (pow(L, 2) * pow(1 / tan(steeringAngle + 0.000001), 2)));
    }

    return turnRadius;
}

/*
 * Function: calcDesiredYawRate
 * ---------------------------------
 * calculates the yaw rate request based on steering angle and velocity of the center of gravity
 * 
 *      steeringAngle: from steering angle sensor
 *      velocityCG: GPS speed [m/s] (from vehicle dynamics module)
 *
 * returns: desired yaw rate
 */
double calcDesiredYawRate(double steeringAngle, double velocityCG) {
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
double calcYawError(double desiredYawRate, double currYawRate) {
    double yawError = desiredYawRate - currYawRate;
    return yawError;
}


/*
 * Function: calcDesiredYawMoment
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
double calcDesiredYawMoment(double yawError, double currYawRate, double desiredYawRate, double prevYawRate, double steeringAngle, double velocityCG, double timestep) {
    double sideSlipAngleCoeff = c_f * l_f - c_r * l_r;
    double yawRateOverVxCoeff = c_f * pow(l_f, 2) + c_r * pow(l_f, 2);
    double steeringAngleCoeff = c_f * l_f;
    double eta = 1.1;

    double turnRadius = calcTurnRadius(steeringAngle);
    double beta = 57.3 * l_r / turnRadius - r * pow(velocityCG, 2) / (K_r * grav * turnRadius);
    double desiredYawRateDerivative = (desiredYawRate - prevYawRate) / timestep;

    double derivTerm = I_z * desiredYawRateDerivative;
    double sideslipTerm = sideSlipAngleCoeff * beta / 57.3; // beta in radians
    double yawOverVxTerm = yawRateOverVxCoeff * currYawRate / velocityCG;
    double steeringAngleTerm = steeringAngleCoeff * steeringAngle; // radians
    double yawErrorTerm = eta * I_z * yawError;

    double desiredYawMoment = derivTerm + sideslipTerm + yawOverVxTerm - steeringAngleTerm + yawErrorTerm;

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
 * Function: calcDesiredTorque
 * --------------------------------
 * calculates the desired torque based off of the torque request, steering angle, and the torque delta
 * 
 *      steeringAngle: from steering angle sensor
 *      torqueRequest: calculated by torque request function
 *      torqueDelta: calculated by torque distribution delta function
 * 
 * returns: desired torque
 */
torque_desired_t calcDesiredTorque(double steeringAngle, double torqueRequest, double torqueDelta) {
    torque_desired_t D;
    double desiredTorqueRR;
    double desiredTorqueRL;

    if (steeringAngle < 0) {
        desiredTorqueRR = 0.5 * (torqueRequest + torqueDelta);
        desiredTorqueRL = 0.5 * (torqueRequest - torqueDelta);
    } else {
        desiredTorqueRR = 0.5 * (torqueRequest - torqueDelta);
        desiredTorqueRL = 0.5 * (torqueRequest + torqueDelta);
    }

    D.desiredTorqueRR = desiredTorqueRR;
    D.desiredTorqueRL = desiredTorqueRL;

    return D;
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
    force_z_t Z;
    Z.F_zrl = (0.5 * mass * grav * l_f / L) - (p_r * accelerationLatitude * mass * heightCG / t_r) + (0.5 * accelerationLongitude * mass * heightCG / L);
    Z.F_zrr = (0.5 * mass * grav * l_f / L) + (p_r * accelerationLatitude * mass * heightCG / t_r) + (0.5 * accelerationLongitude * mass * heightCG / L);

    Z.F_zfl = (0.5 * mass * grav * l_r / L) - (p_f * accelerationLatitude * mass * heightCG / t_r) - (0.5 * accelerationLongitude * mass * heightCG / L);
    Z.F_zfr = (0.5 * mass * grav * l_r / L) + (p_f * accelerationLatitude * mass * heightCG / t_r) - (0.5 * accelerationLongitude * mass * heightCG / L);

    return Z;
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
    Y.F_yfr = (Z.F_zfr / grav) * accelerationLatitude;
    Y.F_yfl = (Z.F_zfl / grav) * accelerationLatitude;
    Y.F_yrr = (Z.F_zrr / grav) * accelerationLatitude;
    Y.F_yrl = (Z.F_zrl / grav) * accelerationLatitude;

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
    torque_max_t M;
    M.maxTorqueRL = (r * sqrt(pow(mu * Z.F_zrl, 2.0) - pow(Y.F_yrl, 2.0))) / G;
    M.maxTorqueRR = (r * sqrt(pow(mu * Z.F_zrr, 2.0) - pow(Y.F_yrr, 2.0))) / G;

    return M;
}
/*
 * Function: checkTractionLimit
 * --------------------------------------
 * calculates the traction limit torque
 * 
 *      D: requested torque struct from calculated desired torque from 
 *      M: max traction limit torques struct from traction limit torque method
 * 
 * returns: struct with the new torques
 */
torque_new_t checkTractionLimit(torque_desired_t D, torque_max_t M) {
    torque_new_t N;
    double newTorqueRR;
    double newTorqueRL;

    if (D.desiredTorqueRR >= 0 && D.desiredTorqueRL >= 0) {

        if (M.maxTorqueRR > D.desiredTorqueRR) {
            if (M.maxTorqueRL > D.desiredTorqueRL) {
                newTorqueRR = D.desiredTorqueRR;
                newTorqueRL = D.desiredTorqueRL;
            } else {
                double Tdiff = fmax(D.desiredTorqueRR - M.maxTorqueRR, D.desiredTorqueRL - M.maxTorqueRL);
                newTorqueRR = D.desiredTorqueRR - Tdiff;
                newTorqueRL = D.desiredTorqueRL - Tdiff;
            }
        } else {
            double Tdiff = fmax(D.desiredTorqueRR - M.maxTorqueRR,D.desiredTorqueRL - M.maxTorqueRL);
            newTorqueRR = D.desiredTorqueRR - Tdiff;
            newTorqueRL = D.desiredTorqueRL - Tdiff;
        }

        if (-newTorqueRL > M.maxTorqueRL) {
            newTorqueRL = -M.maxTorqueRL;
        }

        if (-newTorqueRR > M.maxTorqueRR) {
            newTorqueRR = -M.maxTorqueRR;
        }
    } else if (D.desiredTorqueRR >= 0 && D.desiredTorqueRL < 0) {
        if(M.maxTorqueRR > D.desiredTorqueRR) {
            if(M.maxTorqueRL > -D.desiredTorqueRL) {
                newTorqueRR = D.desiredTorqueRR;
                newTorqueRL = D.desiredTorqueRL;
            } else {
                double Tdiff = -D.desiredTorqueRL - M.maxTorqueRL;
                newTorqueRR = D.desiredTorqueRR - Tdiff;
                newTorqueRL = D.desiredTorqueRL + Tdiff;
            }
        } else {
            double Tdiff = fmax(D.desiredTorqueRR - M.maxTorqueRR, -D.desiredTorqueRL - M.maxTorqueRL);
            newTorqueRR = D.desiredTorqueRR - Tdiff;
            newTorqueRL = fmax(D.desiredTorqueRL - Tdiff, -M.maxTorqueRL);
        }
        
        if (-newTorqueRR > M.maxTorqueRR) {
            double diff = -newTorqueRR - M.maxTorqueRR;
            newTorqueRR = -M.maxTorqueRR;
            newTorqueRL = fmax(newTorqueRL - diff, -M.maxTorqueRL);
        }
    } else if (D.desiredTorqueRR < 0 && D.desiredTorqueRL >= 0) {
        if (M.maxTorqueRL > D.desiredTorqueRL) {
            if (M.maxTorqueRR > -D.desiredTorqueRR) {
                newTorqueRR = D.desiredTorqueRR;
                newTorqueRL = D.desiredTorqueRL;
            } else {
                double Tdiff = -D.desiredTorqueRR - M.maxTorqueRR;
                newTorqueRR = D.desiredTorqueRR + Tdiff;
                newTorqueRL = D.desiredTorqueRL - Tdiff;
            }
        } else {
            double Tdiff = fmax(D.desiredTorqueRL - M.maxTorqueRL, -D.desiredTorqueRR - M.maxTorqueRR);
            newTorqueRL = D.desiredTorqueRL - Tdiff;
            newTorqueRR = fmax(D.desiredTorqueRR - Tdiff, -M.maxTorqueRR);
        }
        
        if (-newTorqueRL > M.maxTorqueRL) {
            double diff = -newTorqueRL - M.maxTorqueRL;
            newTorqueRL = -M.maxTorqueRL;
            newTorqueRR = fmax(newTorqueRR - diff, -M.maxTorqueRR);
        }
    } else if (D.desiredTorqueRR < 0 && D.desiredTorqueRL < 0) {
        if (-M.maxTorqueRL < D.desiredTorqueRL) {
            if (-M.maxTorqueRR < D.desiredTorqueRR) {
                newTorqueRR = D.desiredTorqueRR;
                newTorqueRL = D.desiredTorqueRL;
            } else {
                double Tdiff = -D.desiredTorqueRR - M.maxTorqueRR;
                newTorqueRR = fmin(0,D.desiredTorqueRR + Tdiff);
                newTorqueRL = fmax(-M.maxTorqueRL, D.desiredTorqueRL - Tdiff);
            }
        } else {
            double Tdiff = -D.desiredTorqueRL - M.maxTorqueRL;
            newTorqueRR = fmax(-M.maxTorqueRR, D.desiredTorqueRR - Tdiff);
            newTorqueRL = fmin(0, D.desiredTorqueRL + Tdiff);
        }
    }

    if (newTorqueRR > maxTorque / 2) {
        if (newTorqueRL >= 0) {
            double diff = fmax(newTorqueRR - maxTorque / 2, newTorqueRL - maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.maxTorqueRR), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.maxTorqueRL), -maxTorque / 2);
        } else {
            double diff = fmax(newTorqueRR - maxTorque / 2, -newTorqueRL - maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.maxTorqueRR), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.maxTorqueRL), -maxTorque / 2);
        }
    } else if (newTorqueRL > maxTorque / 2) {
        if (newTorqueRR >= 0) {
            double diff = newTorqueRL - maxTorque / 2;
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.maxTorqueRL), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.maxTorqueRR), -maxTorque / 2);
        } else {
            double diff = fmax(newTorqueRR - maxTorque / 2, -newTorqueRL - maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.maxTorqueRL), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.maxTorqueRR), -maxTorque / 2);
        }
    }

    if (-newTorqueRR > maxTorque / 2) {
        if (newTorqueRL <= 0) {
            double diff = fmax(-newTorqueRR - maxTorque / 2, -newTorqueRL -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR + diff, -M.maxTorqueRR), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL + diff, -M.maxTorqueRL), -maxTorque / 2);
        } else {
            double diff = fmax(-newTorqueRR - maxTorque / 2, newTorqueRL -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR + diff, -M.maxTorqueRR), -maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL - diff, -M.maxTorqueRL), -maxTorque / 2);
        }
    } else if (-newTorqueRL > maxTorque / 2) {
        if (-newTorqueRR >= 0) {
            double diff = -newTorqueRL - maxTorque / 2;
            newTorqueRL = fmax(fmax(newTorqueRL + diff, -M.maxTorqueRL), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR + diff, -M.maxTorqueRR), -maxTorque / 2);
        } else {
            double diff = fmax(newTorqueRR - maxTorque / 2, -newTorqueRL - maxTorque / 2);
            newTorqueRL = fmax(fmax(newTorqueRL + diff, -M.maxTorqueRL), -maxTorque / 2);
            newTorqueRR = fmax(fmax(newTorqueRR - diff, -M.maxTorqueRR), -maxTorque / 2);
        }
    }

    N.newTorqueRL = newTorqueRL;
    N.newTorqueRR = newTorqueRR;

    return N;
}