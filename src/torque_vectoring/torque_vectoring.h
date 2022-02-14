#include <stdio.h>
#include <math.h>

int helper_func(int returnval);
double calculateTorqueRequest(double throttlePosition);
double calculateDesiredYawRate(double steeringAngle, double velocityCG);
double calculateYawError(double desiredYawRate, double currentYawRate);

