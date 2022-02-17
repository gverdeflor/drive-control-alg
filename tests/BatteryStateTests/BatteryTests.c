#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../../src/battery_state/battery_state.h"
#include "../CuTest.h"

// bool GLVSSafetyCheck(double currentVoltage, double currentTemperature);
// double calculateCurrentEnergyConsumption(double packVoltage, double packCurrent, double currentLapTime);
// ------------------------------ GLVSSafetyCheck Tests ------------------------------ //
void GLVSSafetyCheckTest(CuTest* tc) {
    double currentVoltage = 5.0;
    double currentTemperature = 5.0;
    CuAssertTrue(tc, GLVSSafetyCheck(currentVoltage, currentTemperature));

    currentVoltage = 21.0;
    CuAssertTrue(tc, !GLVSSafetyCheck(currentVoltage, currentTemperature));
    
    currentTemperature = 21.0;
    CuAssertTrue(tc, !GLVSSafetyCheck(currentVoltage, currentTemperature));

    currentVoltage = 5.0;
    CuAssertTrue(tc, !GLVSSafetyCheck(currentVoltage, currentTemperature));

    currentTemperature = 7.0;
    CuAssertTrue(tc, GLVSSafetyCheck(currentVoltage, currentTemperature));
}

// ------------------------------ CurrentEnergyConsumption Tests ------------------------------ //
void calculateCurrentEnergyConsumptionTest(CuTest* tc) {
    double packVoltage = 5.0;
    double packCurrent = 5.0;
    double currentLapTime = 5.0;
    CuAssertDblEquals(tc, 125.0, calculateCurrentEnergyConsumption(packVoltage, packCurrent, currentLapTime), 0.0);
    CuAssert(tc, "assert test equality", calculateCurrentEnergyConsumption(packVoltage, packCurrent, currentLapTime) > 124.9 && calculateCurrentEnergyConsumption(packVoltage, packCurrent, currentLapTime) < 125.1);
    
    packVoltage = 1.0;
    currentLapTime = 2.0;
    CuAssertDblEquals(tc, 10.0, calculateCurrentEnergyConsumption(packVoltage, packCurrent, currentLapTime), 0.0);

    packCurrent = 8.0;
    CuAssertDblEquals(tc, 16.0, calculateCurrentEnergyConsumption(packVoltage, packCurrent, currentLapTime), 0.0);
}

// ------------------------------ PerLapCurrentEnergyConsumption Tests ------------------------------ //
void calculatePerLapCurrentEnergyConsumptionTest(CuTest* tc) {
    double currentEnergyConsumption = 5.0;
    double lapsLeft = 5.0;
    CuAssertDblEquals(tc, 1.0, calculatePerLapCurrentEnergyConsumption(currentEnergyConsumption, lapsLeft), 0.0);

    lapsLeft = 1110.0;
    CuAssertDblEquals(tc, .004505, calculatePerLapCurrentEnergyConsumption(currentEnergyConsumption, lapsLeft), 0.000003); // some tolerance since the numbers are large

    currentEnergyConsumption = 1110;
    lapsLeft = 10.0;
    CuAssertDblEquals(tc, 111.0, calculatePerLapCurrentEnergyConsumption(currentEnergyConsumption, lapsLeft), 0.0); // some tolerance since the numbers are large
}



// ------------------------------ DistanceTraveled Tests ------------------------------ //
void calculateDistanceTraveledTest(CuTest* tc) {
    double rearWheelSpeed = 5.0;
    double currentLapTime = 5.0;
    CuAssertDblEquals(tc, 392.699, calculateDistanceTraveled(rearWheelSpeed, currentLapTime), 0.002);
    CuAssert(tc, "range test on distance traveled", 392.699 <= calculateDistanceTraveled(rearWheelSpeed, currentLapTime) && calculateDistanceTraveled(rearWheelSpeed, currentLapTime) <= 392.7);

    CuAssert(tc, "range test on distance traveled 2 -- should fail", 392.699 <= calculateDistanceTraveled(rearWheelSpeed, currentLapTime) && !(calculateDistanceTraveled(rearWheelSpeed, currentLapTime) >= 392.7));
}


// ------------------------------ DistanceTraveled Tests ------------------------------ //
void finishedLapTest(CuTest* tc) {
    double currentLat = 175.0;
    double currentLong = 110.0;
    double currentSpeed = 8.0;
    double currentLapDistance = 7500;

    CuAssertTrue(tc, finishedLap(currentLat, currentLong, currentSpeed, currentLapDistance));

    currentLapDistance = 11000;
    CuAssertTrue(tc, !finishedLap(currentLat, currentLong, currentSpeed, currentLapDistance));
}

// ------------------------------ DistanceTraveled Tests ------------------------------ //
void calculateEnergyConsumptionOffsetTest(CuTest* tc) {
    double estimatedEnergyConsumption = 1.0;
    double perLapCurrentEnergyConsumption = 2.0;

    CuAssertDblEquals(tc, -1.0, calculateEnergyConsumptionOffset(estimatedEnergyConsumption, perLapCurrentEnergyConsumption), 0.0);

    estimatedEnergyConsumption = 1000;
    CuAssert(tc, "correct offset calculation", calculateEnergyConsumptionOffset(estimatedEnergyConsumption, perLapCurrentEnergyConsumption) == 998.0);

    CuAssert(tc, "incorrect offset calculation", (calculateEnergyConsumptionOffset(estimatedEnergyConsumption, perLapCurrentEnergyConsumption) != 998.0) == false);
}


CuSuite* BatteryGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, GLVSSafetyCheckTest);
    SUITE_ADD_TEST(suite, calculateCurrentEnergyConsumptionTest);
    SUITE_ADD_TEST(suite, calculatePerLapCurrentEnergyConsumptionTest);
    SUITE_ADD_TEST(suite, calculateDistanceTraveledTest);
    SUITE_ADD_TEST(suite, finishedLapTest);
    SUITE_ADD_TEST(suite, calculateEnergyConsumptionOffsetTest);

    return suite;
}