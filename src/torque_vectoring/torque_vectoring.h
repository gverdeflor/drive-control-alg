#include <stdio.h>
#include <math.h>

typedef struct ForceZ {
    double F_zrl;
    double F_zrr;
    double F_zfl;
    double F_zfr;
} ForceZStruct;

typedef struct ForceY {
    double F_yfr;
    double F_yfl;
    double F_yrr;
    double F_yrl;
} ForceYStruct;

typedef struct TMax {
    double maxTorqueRL;
    double maxTorqueRR;
} TMaxStruct;

typedef struct DesiredTorque {
    double desiredTorqueRR;
    double desiredTorqueRL;
} DesiredTorqueStruct;

typedef struct NewTorque {
    double newTorqueRR;
    double newTorqueRL;
} NewTorqueStruct;

double calculateTorqueRequest(double throttlePosition, double velocityCG);
double calculateTurnRadius(double steeringAngle);

double calculateDesiredYawRate(double steeringAngle, double velocityCG);
double calculateYawError(double desiredYawRate, double currentYawRate);

double calculateDesiredYawMoment(double yawError, double currentYawRate, double desiredYawRate, double previousYawRate, double steeringAngle, double velocityCG, double timestep);
double calculateTorqueDistributionDelta(double desiredYawMoment);
DesiredTorqueStruct calculateDesiredTorque(double steeringAngle, double torqueRequest, double torqueDelta);

ForceZStruct calculateVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude);
ForceYStruct calculateLateralForces(double accelerationLatitude, ForceZStruct forcesZ);
TMaxStruct calculateTractionLimitTorque(ForceYStruct forcesY, ForceZStruct forcesZ);

NewTorqueStruct tractionLimitCheck(DesiredTorqueStruct desiredTorque, TMaxStruct tMaxes);
