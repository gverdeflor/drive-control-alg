#include <stdio.h>
#include <math.h>

typedef struct ForceZ {
    double F_zrl;
    double F_zrr;
} ForceZStruct;

typedef struct ForceY {
    double F_zfr;
    double F_zfl;
    double F_zrr;
    double F_zrl;
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


int helper_func(int returnval);
double calculateTorqueRequest(double throttlePosition);
double calculateDesiredYawRate(double steeringAngle, double velocityCG);
double calculateYawError(double desiredYawRate, double currentYawRate);

double calculateDesiredYawMoment(double yawError, double steeringAngle);
ForceZStruct calculateVerticalLoadWeightTransfer(double accelerationLongitude, double accelerationLatitude);

TMaxStruct calculateTractionLimitTorque(double F_yrl, double F_yrr, double F_zrl, double F_zrr);
double calculateTorqueDistributionDelta(double desiredYawMoment, double steeringAngle, double velocityCG);
