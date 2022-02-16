#include <stdio.h>
#include <math.h>

typedef struct ForceZ {
    double F_zrl;
    double F_zrr;
} ForceZStruct;

typedef struct TMax {
    double TMax_rl;
    double TMax_rr;
} TMaxStruct;


int helper_func(int returnval);
double calculateTorqueRequest(double throttlePosition);
double calculateDesiredYawRate(double steeringAngle, double velocityCG);
double calculateYawError(double desiredYawRate, double currentYawRate);

double calculateDesiredYawMoment(double yawError, double steeringAngle);
ForceZStruct calculateVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude);

TMaxStruct calculateTractionLimitTorque(double F_yrl, double F_yrr, double F_zrl, double F_zrr);
double calculateTorqueDistributionDelta(double desiredYawMoment, double steeringAngle, double velocityCG);
