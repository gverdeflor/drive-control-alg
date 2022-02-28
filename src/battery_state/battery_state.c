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