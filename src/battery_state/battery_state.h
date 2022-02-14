#include <stdio.h>
#include <math.h>

int helper_func(int returnval);
double GLVSSafetyCheck(double currentVoltage, double currentTemperature);

double calculateTorqueRequest(double throttlePosition);
double calculateDesiredYawRate(double steeringAngle, double velocityCG);
double calculateYawError(double desiredYawRate, double currentYawRate);

