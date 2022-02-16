#pragma once

/*
 * battery_state.h -- public interface to battery state module
 */

#include <stdio.h>
#include <math.h>

/* 
 * indicates whether GLVS safety circuit fault has occurred
 */
bool safetyCircuitCheck(double currVoltage, double currTemperature);

/* 
 * calculates current energy consumption based on pack voltage and current and lap time
 */
double calcCurrEnergyConsumption(double packVoltage, double packCurrent, double currLapTime);

/* 
 * calculates energy consumption allowed per lap given number of laps left
 */
double calcPerLapCurrEnergyConsumption(double currEnergyConsumption, double lapsLeft);

/* 
 * calculates total distance traveled given rear wheel speed and current lap time
 */
double calcDistanceTraveled(double rearWheelSpeed, double currLapTime);

/* 
 * indicates whether lap has been completed
 */
bool finishedLap(double currLatitude, double currLongitude, double currSpeed, double currLapDistance);

/* 
 * calculates energy consumption offset based on estimated energy consumption and per lap energy consumption
 */
double calcEnergyConsumptionError(double estimatedEnergyConsumption, double perLapCurrEnergyConsumption);


