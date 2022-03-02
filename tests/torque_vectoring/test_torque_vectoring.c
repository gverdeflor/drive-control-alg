/* test_torque_vectoring.c ---
 * 
 *
 * Author: Suraj Srivats, Garth Verdeflor
 * Created: Sun Feb 20 11:33:29 2022 (-0400)
 * Version: 1.0  
 *
 *
 * Description: Tests individual torque vectoring algorithm calculations.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../../src/torque_vectoring/torque_vectoring.h"
#include "../CuTest.h"

// --------------------------- Torque Request Tests --------------------------------- //
void calcTorqueRequestTest(CuTest* tc) {
    double throttlePosition = 50;
    double velocityCG = 25;

    CuAssertDblEquals(tc, 89.0, calcTorqueRequest(throttlePosition, velocityCG), 0);

    throttlePosition = 25;
    CuAssertDblEquals(tc, 44.5, calcTorqueRequest(throttlePosition, velocityCG), 0);

    throttlePosition = 75;
    CuAssertDblEquals(tc, 133.5, calcTorqueRequest(throttlePosition, velocityCG), 0);

    throttlePosition = 100;
    CuAssertDblEquals(tc, 178.0, calcTorqueRequest(throttlePosition, velocityCG), 0);

    // Regen Tests
    throttlePosition = 0;
    CuAssertDblEquals(tc, -0.054284, calcTorqueRequest(throttlePosition, velocityCG), 0.0001);

    velocityCG = 10;
    CuAssertDblEquals(tc, -0.054159, calcTorqueRequest(throttlePosition, velocityCG), 0.0001);

    velocityCG = 30;
    CuAssertDblEquals(tc, -0.054284, calcTorqueRequest(throttlePosition, velocityCG), 0.0001);
}

// --------------------------- Turn Radius Tests --------------------------------- //
void calcTurnRadiusTest(CuTest* tc) {
    double steeringAngle = -1.0472; // radians
    CuAssertDblEquals(tc, 1.065236, calcTurnRadius(steeringAngle), 0.0001);

    steeringAngle = 0; // radians
    CuAssertDblEquals(tc, 1749999.999999, calcTurnRadius(steeringAngle), 0.0001);

    steeringAngle = 1.04; // radians
    CuAssertDblEquals(tc, 1.081251, calcTurnRadius(steeringAngle), 0.0001);
}

// --------------------------- Desired Yaw Rate Tests --------------------------------- //
void calcDesiredYawRateTest(CuTest* tc) {
    double steeringAngle = -1;
    double velocityCG = 10;
    CuAssertDblEquals(tc, -8.523310, calcDesiredYawRate(steeringAngle, velocityCG), 0.0001);

    steeringAngle = 0;
    CuAssertDblEquals(tc, -0.000006, calcDesiredYawRate(steeringAngle, velocityCG), 0.0001);

    steeringAngle = 1;
    velocityCG = 35;
    CuAssertDblEquals(tc, 29.831587, calcDesiredYawRate(steeringAngle, velocityCG), 0.0001);
}


// --------------------------- Yaw Error Tests --------------------------------- //
void calcYawErrorTest(CuTest* tc) {
    double desiredYawRate = -2;
    double currYawRate = -1.95;

    CuAssertDblEquals(tc, -0.05, calcYawError(desiredYawRate, currYawRate), 0.0001);

    desiredYawRate = 3.0;
    currYawRate = 2.89;
    CuAssertDblEquals(tc, 0.11, calcYawError(desiredYawRate, currYawRate), 0.0001);
}

// --------------------------- Desired Yaw Moment Tests --------------------------------- //
void calcDesiredYawMomentTest(CuTest* tc) {
    double currYawRate = -2;
    double desiredYawRate = -1.95;
    double prevYawRate = -2.04;
    double steeringAngle = -1.5;
    double velocityCG = 25;
    double timestep = 10;

    double yawError = calcYawError(desiredYawRate, currYawRate);

    CuAssertDblEquals(tc, 13260.220032, calcDesiredYawMoment(yawError, currYawRate, desiredYawRate, prevYawRate, steeringAngle, velocityCG, timestep), 0.0001);

    velocityCG = 35;
    steeringAngle = 1.27;
    desiredYawRate = 1.92;
    prevYawRate = 1.75;
    currYawRate = 1.82;
    yawError = calcYawError(desiredYawRate, currYawRate);
    CuAssertDblEquals(tc, 6415.159445, calcDesiredYawMoment(yawError, currYawRate, desiredYawRate, prevYawRate, steeringAngle, velocityCG, timestep), 0.0001);
}

// --------------------------- Torque Distribution Delta Tests --------------------------------- //
void calcTorqueDistributionDeltaTest(CuTest* tc) {
    double desiredYawMoment = 13247;
    CuAssertDblEquals(tc, 519.962946, calcTorqueDistributionDelta(desiredYawMoment), 0.0001);

    desiredYawMoment = 6415;
    CuAssertDblEquals(tc, 251.797562, calcTorqueDistributionDelta(desiredYawMoment), 0.0001);
}

// ---------------------------  Desired Torque Tests --------------------------------- //
void calcDesiredTorqueTest(CuTest* tc) {
    double steeringAngle = -1;
    double torqueRequest = 133.5;
    double torqueDelta = 519;
    
    torque_desired_t D;
    D.desiredTorqueRR = 326.25;
    D.desiredTorqueRL = -192.75;

    torque_desired_t A = calcDesiredTorque(steeringAngle, torqueRequest, torqueDelta);

    CuAssertDblEquals(tc, D.desiredTorqueRR, A.desiredTorqueRR, 0.0001);
    CuAssertDblEquals(tc, D.desiredTorqueRL, A.desiredTorqueRL, 0.0001);
    
    // Test Two - different Steering Angle
    steeringAngle = 1.75;
    torqueDelta = -55;
    D.desiredTorqueRR = 94.25;
    D.desiredTorqueRL = 39.25;
    
    A = calcDesiredTorque(steeringAngle, torqueRequest, torqueDelta);
    
    CuAssertDblEquals(tc, D.desiredTorqueRR, A.desiredTorqueRR, 0.0001);
    CuAssertDblEquals(tc, D.desiredTorqueRL, A.desiredTorqueRL, 0.0001);
}

// --------------------------- Vertical Load Weight Transfer Tests --------------------------------- //
void calcVerticalLoadWeightTransferTest(CuTest* tc) {
    double accelerationLongitude = -10;
    double accelerationLatitude = -2;

    force_z_t expectedZ;
    expectedZ.F_zrl = 782.922343;
    expectedZ.F_zrr = 782.769085;
    expectedZ.F_zfl = 984.827741;
    expectedZ.F_zfr = 984.680831;

    force_z_t actualZ = calcVerticalLoadWeightTransfer(accelerationLongitude, accelerationLatitude);

    CuAssertDblEquals(tc, expectedZ.F_zrl, actualZ.F_zrl, 0.0001);
    CuAssertDblEquals(tc, expectedZ.F_zrr, actualZ.F_zrr, 0.0001);
    CuAssertDblEquals(tc, expectedZ.F_zfl, actualZ.F_zfl, 0.0001);
    CuAssertDblEquals(tc, expectedZ.F_zfr, actualZ.F_zfr, 0.0001);

    // Test Two
    accelerationLongitude = 7;
    accelerationLatitude = 4;

    expectedZ.F_zrl = 1254.806742;
    expectedZ.F_zrr = 1255.113258;
    expectedZ.F_zfl = 512.493090;
    expectedZ.F_zfr = 512.786910;

    actualZ = calcVerticalLoadWeightTransfer(accelerationLongitude, accelerationLatitude);

    CuAssertDblEquals(tc, expectedZ.F_zrl, actualZ.F_zrl, 0.0001);
    CuAssertDblEquals(tc, expectedZ.F_zrr, actualZ.F_zrr, 0.0001);
    CuAssertDblEquals(tc, expectedZ.F_zfl, actualZ.F_zfl, 0.0001);
    CuAssertDblEquals(tc, expectedZ.F_zfr, actualZ.F_zfr, 0.0001);
}

// --------------------------- Lateral Forces Tests --------------------------------- //
void calcLateralForcesTest(CuTest* tc) {
    double accelerationLatitude = -2;
    force_z_t Z;
    Z.F_zrl = 782.922343;
    Z.F_zrr = 782.769085;
    Z.F_zfl = 984.827741;
    Z.F_zfr = 984.680831;

    force_y_t actualY = calcLateralForces(accelerationLatitude, Z);

    force_y_t expectedY;
    expectedY.F_yrl = -159.454652;
    expectedY.F_yrr = -159.423439;
    expectedY.F_yfl = -200.575915;
    expectedY.F_yfr = -200.545994;

    CuAssertDblEquals(tc, expectedY.F_yrl, actualY.F_yrl, 0.0001);
    CuAssertDblEquals(tc, expectedY.F_yrr, actualY.F_yrr, 0.0001);
    CuAssertDblEquals(tc, expectedY.F_yfl, actualY.F_yfl, 0.0001);
    CuAssertDblEquals(tc, expectedY.F_yfr, actualY.F_yfr, 0.0001);

    // Test Two
    accelerationLatitude = 7.3;

    Z.F_zrl = 1254.806742;
    Z.F_zrr = 1255.113258;
    Z.F_zfl = 512.493090;
    Z.F_zfr = 512.786910;

    actualY = calcLateralForces(accelerationLatitude, Z);

    expectedY.F_yrl = 932.799309;
    expectedY.F_yrr = 933.027167;
    expectedY.F_yfl = 380.977552;
    expectedY.F_yfr = 381.195972;

    CuAssertDblEquals(tc, expectedY.F_yrl, actualY.F_yrl, 0.0001);
    CuAssertDblEquals(tc, expectedY.F_yrr, actualY.F_yrr, 0.0001);
    CuAssertDblEquals(tc, expectedY.F_yfl, actualY.F_yfl, 0.0001);
    CuAssertDblEquals(tc, expectedY.F_yfr, actualY.F_yfr, 0.0001);
}

// --------------------------- Traction Limit Torque Tests --------------------------------- //
void calcTractionLimitTorqueTest(CuTest* tc) {
    force_y_t Y;
    Y.F_yrl = -159.454652;
    Y.F_yrr = -159.423439;
    Y.F_yfl = -200.575915;
    Y.F_yfr = -200.545994;

    force_z_t Z;
    Z.F_zrl = 782.922343;
    Z.F_zrr = 782.769085;
    Z.F_zfl = 984.827741;
    Z.F_zfr = 984.680831;

    torque_max_t expectedTMax;
    expectedTMax.maxTorqueRL = 67.918075;
    expectedTMax.maxTorqueRR = 67.904780;
    
    torque_max_t actualTMax = calcTractionLimitTorque(Y, Z);

    CuAssertDblEquals(tc, expectedTMax.maxTorqueRL, actualTMax.maxTorqueRL, 0.0001);
    CuAssertDblEquals(tc, expectedTMax.maxTorqueRR, actualTMax.maxTorqueRR, 0.0001);

    // Test Two
    Y.F_yrl = 932.799309;
    Y.F_yrr = 933.027167;
    Y.F_yfl = 380.977552;
    Y.F_yfr = 381.195972;

    Z.F_zrl = 1254.806742;
    Z.F_zrr = 1255.113258;
    Z.F_zfl = 512.493090;
    Z.F_zfr = 512.786910;

    expectedTMax.maxTorqueRL = 108.837726;
    expectedTMax.maxTorqueRR = 108.864315;
    
    actualTMax = calcTractionLimitTorque(Y, Z);

    CuAssertDblEquals(tc, expectedTMax.maxTorqueRL, actualTMax.maxTorqueRL, 0.0001);
    CuAssertDblEquals(tc, expectedTMax.maxTorqueRR, actualTMax.maxTorqueRR, 0.0001);
}

// --------------------------- Traction Limit Check Tests --------------------------------- //
void checkTractionLimitTest(CuTest* tc) {
    torque_desired_t D;
    D.desiredTorqueRR = 326.25;
    D.desiredTorqueRL = -192.75;

    torque_max_t M;
    M.maxTorqueRR = 67.904780;
    M.maxTorqueRL = 67.918075;


    torque_new_t expectedNewT;
    expectedNewT.newTorqueRR = 67.904780;
    expectedNewT.newTorqueRL = -451.095220;

    torque_new_t actualNewT = checkTractionLimit(D, M);
    CuAssertDblEquals(tc, expectedNewT.newTorqueRR, actualNewT.newTorqueRR, 0.0001);
    CuAssertDblEquals(tc, expectedNewT.newTorqueRL, actualNewT.newTorqueRL, 0.0001);

    // Test Two - under max
    D.desiredTorqueRR = 80;
    D.desiredTorqueRL = 87;

    M.maxTorqueRR = 90;
    M.maxTorqueRL = 90;

    expectedNewT.newTorqueRR = 80;
    expectedNewT.newTorqueRL = 87;

    actualNewT = checkTractionLimit(D, M);
    CuAssertDblEquals(tc, expectedNewT.newTorqueRR, actualNewT.newTorqueRR, 0.0001);
    CuAssertDblEquals(tc, expectedNewT.newTorqueRL, actualNewT.newTorqueRL, 0.0001);

    // Test Three - end rescale
    D.desiredTorqueRR = 120;
    D.desiredTorqueRL = 120;

    M.maxTorqueRR = 140;
    M.maxTorqueRL = 140;

    expectedNewT.newTorqueRR = 89;
    expectedNewT.newTorqueRL = 89;

    actualNewT = checkTractionLimit(D, M);
    CuAssertDblEquals(tc, expectedNewT.newTorqueRR, actualNewT.newTorqueRR, 0.0001);
    CuAssertDblEquals(tc, expectedNewT.newTorqueRL, actualNewT.newTorqueRL, 0.0001);
}

CuSuite* TorqueVectoringGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, calcTorqueRequestTest);
    SUITE_ADD_TEST(suite, calcTurnRadiusTest);
    SUITE_ADD_TEST(suite, calcDesiredYawRateTest);
    SUITE_ADD_TEST(suite, calcYawErrorTest);
    SUITE_ADD_TEST(suite, calcDesiredYawMomentTest);
    SUITE_ADD_TEST(suite, calcTorqueDistributionDeltaTest);
    SUITE_ADD_TEST(suite, calcDesiredTorqueTest);
    SUITE_ADD_TEST(suite, calcVerticalLoadWeightTransferTest);
    SUITE_ADD_TEST(suite, calcLateralForcesTest);
    SUITE_ADD_TEST(suite, calcTractionLimitTorqueTest);
    SUITE_ADD_TEST(suite, checkTractionLimitTest);

    return suite;
}
