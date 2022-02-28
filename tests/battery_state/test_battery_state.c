/* test_battery_state.c ---
 * 
 *
 * Author: Suraj Srivats, Garth Verdeflor
 * Created: Tues Feb 15 11:33:29 2022 (-0400)
 * Version: 1.0  
 *
 *
 * Description: Tests individual battery state algorithm calculations.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "../../src/battery_state/battery_state.h"
#include "../CuTest.h"

// ------------------------------ Safety Circuit Tests ------------------------------ //
void safetyCircuitCheckTest(CuTest* tc) {
    double currVoltage = 5.0;
    double currTemperature = 5.0;
    CuAssertTrue(tc, safetyCircuitCheck(currVoltage, currTemperature));

    currVoltage = 21.0;
    CuAssertTrue(tc, !safetyCircuitCheck(currVoltage, currTemperature));
    
    currTemperature = 21.0;
    CuAssertTrue(tc, !safetyCircuitCheck(currVoltage, currTemperature));

    currVoltage = 5.0;
    CuAssertTrue(tc, !safetyCircuitCheck(currVoltage, currTemperature));

    currTemperature = 7.0;
    CuAssertTrue(tc, safetyCircuitCheck(currVoltage, currTemperature));
}


// ------------------------------ Current Energy Consumption Tests ------------------------------ //
void calcCurrEnergyConsumptionTest(CuTest* tc) {
    double packVoltage = 5.0;
    double packCurrent = 5.0;
    double currLapTime = 5.0;
    CuAssertDblEquals(tc, 125.0, calcCurrEnergyConsumption(packVoltage, packCurrent, currLapTime), 0.0);
    CuAssert(tc, "assert test equality", calcCurrEnergyConsumption(packVoltage, packCurrent, currLapTime) > 124.9 && calcCurrEnergyConsumption(packVoltage, packCurrent, currLapTime) < 125.1);
    
    packVoltage = 1.0;
    currLapTime = 2.0;
    CuAssertDblEquals(tc, 10.0, calcCurrEnergyConsumption(packVoltage, packCurrent, currLapTime), 0.0);

    packCurrent = 8.0;
    CuAssertDblEquals(tc, 16.0, calcCurrEnergyConsumption(packVoltage, packCurrent, currLapTime), 0.0);
}


// ------------------------------ Per Lap Current Energy Consumption Tests ------------------------------ //
void calcPerLapCurrEnergyConsumptionTest(CuTest* tc) {
    double currEnergyConsumption = 5.0;
    double lapsLeft = 5.0;
    CuAssertDblEquals(tc, 1.0, calcPerLapCurrEnergyConsumption(currEnergyConsumption, lapsLeft), 0.0);

    lapsLeft = 1110.0;
    CuAssertDblEquals(tc, .004505, calcPerLapCurrEnergyConsumption(currEnergyConsumption, lapsLeft), 0.000003); // some tolerance since the numbers are large

    currEnergyConsumption = 1110;
    lapsLeft = 10.0;
    CuAssertDblEquals(tc, 111.0, calcPerLapCurrEnergyConsumption(currEnergyConsumption, lapsLeft), 0.0); // some tolerance since the numbers are large
}


// ------------------------------ Distance Traveled Tests ------------------------------ //
void calcDistanceTraveledTest(CuTest* tc) {
    double rearWheelSpeed = 5.0;
    double currLapTime = 5.0;

    CuAssertDblEquals(tc, 392.699, calcDistanceTraveled(rearWheelSpeed, currLapTime), 0.002);
    CuAssert(tc, "range test on distance traveled", 392.699 <= calcDistanceTraveled(rearWheelSpeed, currLapTime) && calcDistanceTraveled(rearWheelSpeed, currLapTime) <= 392.7);
    CuAssert(tc, "range test on distance traveled 2 -- should fail", 392.699 <= calcDistanceTraveled(rearWheelSpeed, currLapTime) && !(calcDistanceTraveled(rearWheelSpeed, currLapTime) >= 392.7));
}


// ------------------------------ Finished Lap Tests ------------------------------ //
void finishedLapTest(CuTest* tc) {
    double currLatitude = 175.0;
    double currLongitude = 110.0;
    double currSpeed = 8.0;
    double currLapDistance = 7500;

    CuAssertTrue(tc, finishedLap(currLatitude, currLongitude, currSpeed, currLapDistance));

    currLapDistance = 11000;
    CuAssertTrue(tc, !finishedLap(currLatitude, currLongitude, currSpeed, currLapDistance));
}

// ------------------------------ Energy Consumption Error Tests ------------------------------ //
void calcEnergyConsumptionErrorTest(CuTest* tc) {
    double estimatedEnergyConsumption = 1.0;
    double perLapCurrEnergyConsumption = 2.0;

    CuAssertDblEquals(tc, -1.0, calcEnergyConsumptionError(estimatedEnergyConsumption, perLapCurrEnergyConsumption), 0.0);

    estimatedEnergyConsumption = 1000;
    CuAssert(tc, "correct offset calculation", calcEnergyConsumptionError(estimatedEnergyConsumption, perLapCurrEnergyConsumption) == 998.0);

    CuAssert(tc, "incorrect offset calculation", (calcEnergyConsumptionError(estimatedEnergyConsumption, perLapCurrEnergyConsumption) != 998.0) == false);
}

CuSuite* BatteryStateGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, safetyCircuitCheckTest);
    SUITE_ADD_TEST(suite, calcCurrEnergyConsumptionTest);
    SUITE_ADD_TEST(suite, calcPerLapCurrEnergyConsumptionTest);
    SUITE_ADD_TEST(suite, calcDistanceTraveledTest);
    SUITE_ADD_TEST(suite, finishedLapTest);
    SUITE_ADD_TEST(suite, calcEnergyConsumptionErrorTest);

    return suite;
}