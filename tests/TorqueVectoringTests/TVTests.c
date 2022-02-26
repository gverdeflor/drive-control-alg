#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../../src/torque_vectoring/torque_vectoring.h"
#include "../CuTest.h"

// --------------------------- Torque Request Tests --------------------------------- //
void calculateTorqueRequestTest(CuTest* tc) {
    double throttlePosition = .5;
    CuAssertDblEquals(tc, calculateTorqueRequest(throttlePosition), 90.0, 0);

    throttlePosition = .25;
    CuAssertDblEquals(tc, calculateTorqueRequest(throttlePosition), 45.0, 0);

    throttlePosition = .75;
    CuAssertDblEquals(tc, calculateTorqueRequest(throttlePosition), 135.0, 0);

    throttlePosition = 1;
    CuAssertDblEquals(tc, calculateTorqueRequest(throttlePosition), 180.0, 0);
}


// --------------------------- Desired Yaw Rate Tests --------------------------------- //
void calculateDesiredYawRateTest(CuTest* tc) {
    double steeringAngle = 25.0;
    double velocityCG = 45.0;
    CuAssertDblEquals(tc, calculateDesiredYawRate(steeringAngle, velocityCG), -8.15, .01);

    velocityCG = 15.0;
    CuAssertDblEquals(tc, calculateDesiredYawRate(steeringAngle, velocityCG), -27.21, .01);
}


// --------------------------- Calculate Yaw Error Tests --------------------------------- //
void calculateYawErrorTest(CuTest* tc) {
    double desiredYawRate = 15.0;
    double currentYawRate = 10;

    CuAssertDblEquals(tc, calculateYawError(desiredYawRate, currentYawRate), 5.0, .01);

    desiredYawRate = -3.0;
    currentYawRate = 7.0;
    CuAssertDblEquals(tc, calculateYawError(desiredYawRate, currentYawRate), -10.0, .01);
}

// --------------------------- Calculate Desired Yaw Moment Tests --------------------------------- //
void calculateDesiredYawMomentTest(CuTest* tc) {
    double yawError = 25.0;
    double steeringAngle = 80.0;

    CuAssertDblEquals(tc, calculateDesiredYawMoment(yawError, steeringAngle), 5.0, .01);

    yawError = -3.0;
    steeringAngle = 7.0;
    CuAssertDblEquals(tc, calculateDesiredYawMoment(yawError, steeringAngle), -10.0, .01);

    yawError = -3.0;
    steeringAngle = 7.0;
    CuAssert(tc, "not equals yaw moment test", calculateDesiredYawMoment(yawError, steeringAngle) != 10.0);
}

CuSuite* TVGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, calculateTorqueRequestTest);
    SUITE_ADD_TEST(suite, calculateDesiredYawRateTest);
    SUITE_ADD_TEST(suite, calculateYawErrorTest);
    SUITE_ADD_TEST(suite, calculateDesiredYawMomentTest);

    return suite;
}

