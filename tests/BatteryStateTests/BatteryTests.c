#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../../src/battery_state/battery_state.h"
#include "../CuTest.h"

bool GLVSSafetyCheck(double currentVoltage, double currentTemperature);

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

CuSuite* BatteryGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, GLVSSafetyCheckTest);

    return suite;
}