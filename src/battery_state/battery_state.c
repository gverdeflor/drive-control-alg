#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define minAcceptableVoltage   0    // UPDATE -- V (volts)
#define maxAcceptableVoltage 10     // UPDATE -- V (volts)
#define wheelDiameter 5             // UPDATE
#define minLapSpeed 5               // UPDATE
#define maxLapSpeed 10              // UPDATE
#define minLatitude 100             // UPDATE
#define maxLatitude 200             // UPDATE
#define minLongitude 100            // UPDATE
#define maxLongitude 200             // UPDATE
#define minReasonableLapDistance 5000   // UPDATE
#define maxReasonableLapDistance 10000 // UPDATE

int helper_func(int returnval) {
    return returnval;
}

/*
 * Function: GLVSSafetyCheck
 * -------------------------
 * given the current voltage and current temperature, returns whether it passes the GLVSSafetyCheck
 * 
 * currentVoltage: voltage (from BMS)
 * currentTemperature: degrees F/C (from BMS)
 * 
 * returns: true if it passes safety check, false if not
 */

// ADD TEMPERATURE HERE
double GLVSSafetyCheck(double currentVoltage, double currentTemperature) {
    if ((minAcceptableVoltage <= currentVoltage <= maxAcceptableVoltage)) { 
        return true;
    } else {
        return false;
    }
}



/*
 * Function: calculateCurrentEnergyConsumption
 * -------------------------------------------
 * calculates energy consumption based on pack voltage, pack current, and the current lap time
 * 
 * packVoltage: voltage (from BMS)
 * packCurrent: current (from BMS)
 * currentLapTime: seconds (from VDM?)
 * 
 * returns: the current energy consumption rate
 */
double calculateCurrentEnergyConsumption(double packVoltage, double packCurrent, double currentLapTime) {
    double currentEnergyConsumption = packVoltage * packCurrent * currentLapTime;
    return currentEnergyConsumption;
}


/*
 * Function: calculatePerLapCurrentEnergyConsumption
 * -------------------------------------------------
 * calculates the energy consumption allowed per lap given the number of laps left
 * 
 * currentEnergyConsumption: voltage
 * lapsLeft: int number of laps left
 * 
 * returns: the current per lap energy consumption rate
 */
double calculatePerLapCurrentEnergyConsumption(double currentEnergyConsumption, double lapsLeft) {
    double perLapCurrentEnergyConsumption = currentEnergyConsumption / lapsLeft;
    return perLapCurrentEnergyConsumption;
}

/*
 * Function: calculateDistanceTraveled
 * -----------------------------------
 * calculate the total distance traveled given the rear wheel speed and current lap time
 * 
 * rearWheelSpeed: m/s (from FWSS)
 * currentLapTime: seconds (from VDM?)
 * 
 * returns: the current energy consumption rate
 */
double calculateDistanceTraveled(double rearWheelSpeed, double currentLapTime) {
    double distanceTraveled = (rearWheelSpeed * M_PI * wheelDiameter) * currentLapTime;
    return distanceTraveled;
}

/*
 * Function: finishedLap
 * ---------------------
 * calculates whether or not a lap has been completed
 * 
 * currentLatitude: bits (from VDM)
 * currentLongitude: bits (from VDM)
 * currentSpeed: gps (from VDM)
 * currentLapDistance: m (distanceTraveled)
 * 
 * returns: true if the lap has been finished, false if the lap has not been finished
 */
bool finishedLap(double currentLatitude, double currentLongitude, double currentSpeed, double currentLapDistance) {
    if ((minLapSpeed <= currentSpeed <= maxLapSpeed) && (minLatitude <= currentLatitude <= maxLatitude)
        && (minLongitude <= currentLongitude <= maxLongitude) && (minReasonableLapDistance <= currentLapDistance <= maxReasonableLapDistance)) {
            return true;
        } else {
            return false;
        }
}

/*
 * Function: calculateEnergyConsumptionOffset
 * ------------------------------------------
 * calculates the energy consumption offset based on estimated energy consumption and the per lap energy consumption
 * 
 * estimatedEnergyConsumption: watts (from calculateCurrentEnergyConsumption)
 * perLapCurrentEnergyConsumption: watts (from calculatePerLapCurrentEnergyConsumption)
 * 
 * returns: the current energy consumption rate
 */
double calculateEnergyConsumptionOffset(double estimatedEnergyConsumption, double perLapCurrentEnergyConsumption) {
    double perLapEnergyOffset = estimatedEnergyConsumption - perLapCurrentEnergyConsumption;
    return perLapEnergyOffset;
}

