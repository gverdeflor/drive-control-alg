#include <stdio.h>
#include <math.h>
#include "torque_vectoring.h"

#define maxTorque   178         // Nm (Newton * meters) - motor datasheet (89 from each motor)
#define carWeight   .6          // percent weight in the rear (estimated)
#define L           1.75        // m (meters) - wheel base to wheel base length
#define l_r         (1 - carWeight) * L         // m (meters) - length from CG to rear
#define l_f         (carWeight) * L        // m (meters) - length from CG to front
#define mass        360         // 360 kg - estimated from previous car
#define c_f         529         // N / deg (Newtons per degree) - cornerning stiffness (front) - estimated from similar SAE car --> needs to be converted to radians
#define c_r         633         // N / deg (Newtons per degree) - cornering stiffness (rear) - estimated from similar SAE car --> needs to be converted to radians
#define K_u         (l_r * mass / (c_f * L)) - (l_f * mass / (c_r * L))     // K constant
#define t_f         1.35         // m (meters) - distance between two front wheels
#define t_r         1.3          // m (meters) - distance between two rear wheels

#define r           .28575      // m (meters) - wheel radius
#define mu          1.7         // constant - coefficient of friction (for now)
#define G           5.6         // constant - gear ratio between motor and wheel
#define heightCG    .27         // m (meters) - height of center of gravity (off the ground)
#define h_s         .28         // roll (?)
#define K_r         570.0197    // roll stiffness rear
#define K_f         546.4094    // roll stiffness front
#define p_r         h_s * (K_r / (K_f * K_r))   // roll distribution rear (units?)
#define p_f         h_s * (K_f / (K_f * K_r))   // roll distribution rear (units?)
#define I_z         120         // Kg * (m) ^ 2 - moment of inertia in the z axis
#define grav        9.82        // m / (s) ^ 2    // gravity


/*
 * Function: calculateTorqueRequest
 * --------------------------------
 * calculates the torque request based on throttle position and max torque
 * in the case that throttlePosition = 0, it does regen braking
 * 
 * throttlePosition: pedal compression [0 - 100] (from throttle position sensor)
 * velocityCG: GPS speed m/s from the VDM
 * 
 * returns: the requested torque as a percent of the maximum available torque
 */
double calculateTorqueRequest(double throttlePosition, double velocityCG) {
    double requestedTorque;
    requestedTorque = maxTorque * (throttlePosition / 100);

    if (requestedTorque == 0) {
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

        double forceDueToMotor = (1 - exp(- velocityCG / .8) * ((targetA * mass) - dragForce - rollingForce));
        double torqueAtWheel = r * forceDueToMotor;

        double torqueAtMotor;
        if (((torqueAtWheel / G / .94) * motorRadSec) < maxP * 1000) {
            torqueAtMotor = torqueAtWheel / G / .94;
        } else {
            torqueAtMotor = 1000 * maxP / motorRadSec;
        }

        requestedTorque = - torqueAtMotor;
    }
    return requestedTorque;
}

/*
 * Function: calculateTurnRadius
 * -----------------------------
 * calculates the turn radius based on the steering angle
 * 
 * steeringAngle: from steering angle sensor
 * returns: the calculated turn radius
 */
double calculateTurnRadius(double steeringAngle) {
    double turnRadius;
    if (steeringAngle != 0) {
        turnRadius = sqrt(pow(t_f/4, 2) + (pow(L, 2) * pow(1 / tan(steeringAngle), 2)));
    } else {
        turnRadius = sqrt(pow(t_f/4, 2) + (pow(L, 2) * pow(1 / tan(steeringAngle + .000001), 2)));
    }

    return turnRadius;
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
    double desiredYawRate;
    double turnRadius = calculateTurnRadius(steeringAngle);

    if (steeringAngle > 0) {
        desiredYawRate = (velocityCG / turnRadius);
    } else {
        desiredYawRate = - (velocityCG / turnRadius);
    }

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
 * -----------------------------------
 * calculates the yaw error based on the difference between the desired rate and the current rate
 * 
 * desiredYawRate: desired yaw rate based on driver input (steering angle sensor + vehicle speed)
 * currentYawRate: current yaw rate of the vehicle - from vehicle dynamics module (VDM)
 * returns: difference between the desired rate and the current rate
 */
double calculateDesiredYawMoment(double yawError, double currentYawRate, double desiredYawRate, double previousYawRate, double steeringAngle, double velocityCG, double timestep) {
    double sideSlipAngleCoeff = c_f * l_f - c_r * l_r;
    double yawRateOverVxCoeff = c_f * pow(l_f, 2) + c_r * pow(l_f, 2);
    double steeringAngleCoeff = c_f * l_f;
    double eta = 1.1;

    double turnRadius = calculateTurnRadius(steeringAngle);
    double beta = 57.3 * l_r / turnRadius - r * pow(velocityCG, 2) / (K_r * grav * turnRadius);
    double desiredYawRateDerivative = (desiredYawRate - previousYawRate) / timestep;

    double derivTerm = I_z * desiredYawRateDerivative;
    double sideslipTerm = sideSlipAngleCoeff * beta; // beta
    double yawOverVxTerm = yawRateOverVxCoeff * currentYawRate / velocityCG;
    double steeringAngleTerm = steeringAngleCoeff * steeringAngle; // radians
    double yawErrorTerm = eta * I_z * yawError;

    double desiredYawMoment = derivTerm + sideslipTerm + yawOverVxTerm - steeringAngleTerm + yawErrorTerm;

    return desiredYawMoment;
}

/*
 * Function: calculateTorqueDistributionDelta
 * ------------------------------------------
 * calculates the torque delta based off of the desired yaw moment
 * 
 * desiredYawMoment: from the calculateDesiredYawMoment function (double)
 * 
 * returns: torque delta
 */
double calculateTorqueDistributionDelta(double desiredYawMoment) {
    double torqueDelta = desiredYawMoment * r / (t_r * G);

    return torqueDelta;
}

/*
 * Function: calculateDesiredTorque
 * --------------------------------
 * calculates the desired torque based off of the torque request, steering angle, and the torque delta
 * 
 * steeringAngle: from the SA Sensor
 * torqueRequest: calculated by torque request function
 * torqueDelta: calculated by torque distribution delta function
 * 
 * returns: desired torque
 */
DesiredTorqueStruct calculateDesiredTorque(double steeringAngle, double torqueRequest, double torqueDelta) {
    DesiredTorqueStruct desiredTorque;
    double desiredTorqueRR;
    double desiredTorqueRL;

    if (steeringAngle < 0) {
        desiredTorqueRR = .5 * (torqueRequest + torqueDelta);
        desiredTorqueRL = .5 * (torqueRequest - torqueDelta);
    } else {
        desiredTorqueRR = .5 * (torqueRequest - torqueDelta);
        desiredTorqueRL = .5 * (torqueRequest + torqueDelta);
    }

    desiredTorque.desiredTorqueRR = desiredTorqueRR;
    desiredTorque.desiredTorqueRL = desiredTorqueRL;

    return desiredTorque;
}

/*
 * Function: calculateVerticalLoadWeightTransfer
 * ---------------------------------------------
 * calculates the weight transfer of the vertical load
 * 
 * accelerationLongitude: from the VDM
 * accelerationLatitude: from the VDM
 * 
 * returns: struct with the four z forces
 */
ForceZStruct calculateVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude) {
    ForceZStruct forces;
    forces.F_zrl = (.5 * mass * grav * l_f / L) - (p_r * accelerationLatitude * mass * heightCG / t_r) + (.5 * accelerationLongitude * mass * heightCG / L);
    forces.F_zrr = (.5 * mass * grav * l_f / L) + (p_r * accelerationLatitude * mass * heightCG / t_r) + (.5 * accelerationLongitude * mass * heightCG / L);

    forces.F_zfl = (.5 * mass * grav * l_r / L) - (p_f * accelerationLatitude * mass * heightCG / t_r) - (.5 * accelerationLongitude * mass * heightCG / L);
    forces.F_zfr = (.5 * mass * grav * l_r / L) + (p_f * accelerationLatitude * mass * heightCG / t_r) - (.5 * accelerationLongitude * mass * heightCG / L);

    return forces;
}

/*
 * Function: calculateLateralForces
 * --------------------------------
 * calculates the lateral forces
 * 
 * accelerationLatitude: from the VDM
 * forcesZ: from calculateVerticalLoadWeightTransfer method
 * 
 * returns: struct with the four y forces
 */
ForceYStruct calculateLateralForces(double accelerationLatitude, ForceZStruct forcesZ) {
    ForceYStruct forces;
    forces.F_yfr = (forcesZ.F_zfr / grav) * accelerationLatitude;
    forces.F_yfl = (forcesZ.F_zfl / grav) * accelerationLatitude;
    forces.F_yrr = (forcesZ.F_zrr / grav) * accelerationLatitude;
    forces.F_yrl = (forcesZ.F_zrl / grav) * accelerationLatitude;

    return forces;
}

/*
 * Function: calculateTractionLimitTorque
 * --------------------------------------
 * calculates the traction limit torque
 * 
 * YForces: from the calculate lateral forces method
 * ZForces: from the calculate vertical load weight transfer method
 * 
 * returns: struct with the max traction limit torques
 */
TMaxStruct calculateTractionLimitTorque(ForceYStruct YForces, ForceZStruct ZForces) {
    TMaxStruct tMaxes;
    tMaxes.maxTorqueRL = (r * sqrt(pow(mu * ZForces.F_zrl, 2.0) - YForces.F_yrl)) / G;
    tMaxes.maxTorqueRR = (r * sqrt(pow(mu * ZForces.F_zrr, 2.0) - YForces.F_yrr)) / G;

    return tMaxes;
}

/*
 * Function: tractionLimitCheckTest
 * --------------------------------------
 * calculates the traction limit torque
 * 
 * desiredTorque: desired torque struct from calculated desired torque from 
 * tMaxes: max traction limit torques struct from traction limit torque method
 * 
 * returns: struct with the new torques
 */
NewTorqueStruct tractionLimitCheck(DesiredTorqueStruct desiredTorque, TMaxStruct tMaxes) {
    NewTorqueStruct newTorque;
    double newTorqueRR;
    double newTorqueRL;
    
    if ((tMaxes.maxTorqueRR > desiredTorque.desiredTorqueRR) && (tMaxes.maxTorqueRL > desiredTorque.desiredTorqueRL)) {
        newTorqueRR = desiredTorque.desiredTorqueRR;
        newTorqueRL = desiredTorque.desiredTorqueRL;
    } else {
        double TDiff = fmax(desiredTorque.desiredTorqueRR - tMaxes.maxTorqueRR, desiredTorque.desiredTorqueRL - tMaxes.maxTorqueRL);
        newTorqueRR = desiredTorque.desiredTorqueRR - TDiff;
        newTorqueRL = desiredTorque.desiredTorqueRL - TDiff;
    }

    if ((newTorqueRR > (maxTorque / 2)) || (newTorqueRL > (maxTorque / 2))) {
        double diff = fmax(newTorqueRR - maxTorque / 2, newTorqueRL - maxTorque / 2);
        newTorqueRR = newTorqueRR - diff;
        newTorqueRL = newTorqueRL - diff;
    }

    newTorque.newTorqueRR = newTorqueRR;
    newTorque.newTorqueRL = newTorqueRL;

    return newTorque;
}