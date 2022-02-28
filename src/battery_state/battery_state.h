#pragma once

/*
 * battery_state.h -- public interface to battery state module
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* ALGORITHM PARAMETERS */
#define minVoltage 50               // V        * mimimum voltage pack should be discharged to *
#define maxVoltage 250              // V        * nominal voltage of fully charged pack *
#define minTemperature 10           // deg C    * from Sony VTC5A 18650 datasheet *
#define maxTemperature 50           // deg C    * from Sony VTC5A 18650 datasheet *
#define wheelDiameter 0.5207        // m        * 20-in Hoosier wheels *
#define minLapSpeed 6.7             // m/s      * 15 mph minimum speed crossing finish line *
#define maxLapSpeed 31.3            // m/s      * 70 mph maximum speed crossing finish line *
#define minLatitude 100             // deg      * DEFINES GEOZONE *
#define maxLatitude 100             // deg      * DEFINES GEOZONE *
#define minLongitude 100            // deg      * DEFINES GEOZONE *
#define maxLongitude 100            // deg      * DEFINES GEOZONE *
#define minLapDistance 950          // m        * lap distance taking narrowest turns *
#define maxLapDistance 1050         // m        * lap distance taking widest turns *

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


