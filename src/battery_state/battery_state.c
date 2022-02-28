/* battery_state.c ---
 * 
 *
 * Author: Suraj Srivats, Garth Verdeflor
 * Created: Mon Feb 14 11:33:29 2022 (-0400)
 * Version: 1.0  
 *
 *
 * Description: Implements battery state algorithm.
 *
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

// USING DUMMY VARIABLES, NEED TO UPDATE!
#define minVoltage 10           // V
#define maxVoltage 290          // V
#define minTemperature 10       // deg C
#define maxTemperature 290      // deg C
#define wheelDiameter 5         // m
#define minLapSpeed 5           // m/s
#define maxLapSpeed 40          // m/s
#define minLatitude 100         // deg
#define maxLatitude 100         // deg
#define minLongitude 100        // deg
#define maxLongitude 100        // deg
#define minLapDistance 5000     // m
#define maxLapDistance 10000    // m

//------------------------ safetyCircuitCheck ----------------------
// Description:   checks if GLVS safety circuit fault has occurred
// Inputs:        current pack voltage and temperature
// Outputs:       true if safety check passed, false otherwise
//------------------------------------------------------------------
bool safetyCircuitCheck(double currVoltage, double currTemperature) {
    // Check pack voltage and temperature limits
    if ( ((minVoltage <= currVoltage) && (currVoltage <= maxVoltage)) && ((minTemperature <= currTemperature) && (currTemperature <= maxTemperature)) ) {
        return true;
    } else {
        return false;
    }
}

//----------------- calcCurrEnergyConsumption ----------------------
// Description:   calculates current energy consumption 
// Inputs:        pack voltage, pack current, and lap time
// Outputs:       current energy consumption
//------------------------------------------------------------------
double calcCurrEnergyConsumption(double packVoltage, double packCurrent, double currLapTime) {
    double currEnergyConsumption = packVoltage * packCurrent * currLapTime;
    return currEnergyConsumption;
}

//---------------- calcPerLapCurrEnergyConsumption -----------------
// Description:   calculates energy consumption allowed per lap
// Inputs:        current energy consumption, laps left in endurance event
// Outputs:       per lap energy consumption recommendation
//------------------------------------------------------------------
double calcPerLapCurrEnergyConsumption(double currEnergyConsumption, double lapsLeft) {
    double perLapCurrEnergyConsumption = currEnergyConsumption / lapsLeft;
    return perLapCurrEnergyConsumption;
}

//------------------- calcDistanceTraveled -------------------------
// Description:   calculates total distance traveled in event 
// Inputs:        rear wheel speed and current lap time
// Outputs:       current distance traveled
//------------------------------------------------------------------
double calcDistanceTraveled(double rearWheelSpeed, double currLapTime) {
    double distanceTraveled = (rearWheelSpeed * M_PI * wheelDiameter) * currLapTime;
    return distanceTraveled;
}

//-------------------------finishedLap -----------------------------
// Description:   indicates whether lap has been completed
// Inputs:        current latitude, longitude, speed, and lap distance
// Outputs:       true if lap has been finished, false otherwise
//------------------------------------------------------------------
bool finishedLap(double currLatitude, double currLongitude, double currSpeed, double currLapDistance) {
    if ( ((minLapSpeed <= currSpeed) && (currSpeed <= maxLapSpeed)) &&   // check for appropriate speed crossing finish line
        ((minLapSpeed <= currSpeed) && (currSpeed <= maxLapSpeed)) &&    // check for latitude associated with finish line
        ((minLapSpeed <= currSpeed) && (currSpeed <= maxLapSpeed)) &&    // check for longitude associated with finish line
        ((minLapSpeed <= currSpeed) && (currSpeed <= maxLapSpeed)) ) {   // check for appropriate distance traveled for a lap
            return true;
        } else {
            return false;
        }
}

//------------------- calcEnergyConsumptionError -------------------
// Description:   calculates energy consumption error
// Inputs:        estimated energy consumption and per lap current energy consumption
// Outputs:       energy consunmption error for current lap
//------------------------------------------------------------------
bool calcEnergyConsumptionError(double estimatedEnergyConsumption, double perLapCurrEnergyConsumption) {
    double perLapEnergyError = estimatedEnergyConsumption - perLapCurrEnergyConsumption;
    return perLapEnergyError;
}