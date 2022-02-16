#include <stdio.h>
#include <math.h>
#include <stdbool.h>

int helper_func(int returnval);
bool GLVSSafetyCheck(double currentVoltage, double currentTemperature);
double calculateCurrentEnergyConsumption(double packVoltage, double packCurrent, double currentLapTime);
double calculatePerLapCurrentEnergyConsumption(double currentEnergyConsumption, double lapsLeft);
double calculateDistanceTraveled(double rearWheelSpeed, double currentLapTime);
bool finishedLap(double currentLatitude, double currentLongitude, double currentSpeed, double currentLapDistance);

double calculateEnergyConsumptionOffset(double estimatedEnergyConsumption, double perLapCurrentEnergyConsumption);