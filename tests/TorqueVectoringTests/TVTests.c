#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../../src/torque_vectoring/torque_vectoring.h"
#include "../CuTest.h"

// --------------------------- Torque Request Tests --------------------------------- //
void calculateTorqueRequestTest(CuTest* tc) {
    double throttlePosition = 50;
    double velocityCG = 25;

    CuAssertDblEquals(tc, 89.0, calculateTorqueRequest(throttlePosition, velocityCG), 0);

    throttlePosition = 25;
    CuAssertDblEquals(tc, 44.5, calculateTorqueRequest(throttlePosition, velocityCG), 0);

    throttlePosition = 75;
    CuAssertDblEquals(tc, 133.5, calculateTorqueRequest(throttlePosition, velocityCG), 0);

    throttlePosition = 100;
    CuAssertDblEquals(tc, 178.0, calculateTorqueRequest(throttlePosition, velocityCG), 0);

    // Regen Tests
    throttlePosition = 0;
    CuAssertDblEquals(tc, -0.054284, calculateTorqueRequest(throttlePosition, velocityCG), .0001);

    velocityCG = 10;
    CuAssertDblEquals(tc, -0.054159, calculateTorqueRequest(throttlePosition, velocityCG), .0001);

    velocityCG = 30;
    CuAssertDblEquals(tc, -0.054284, calculateTorqueRequest(throttlePosition, velocityCG), .0001);
}

// --------------------------- Turn Radius Tests --------------------------------- //
void calculateTurnRadiusTest(CuTest* tc) {
    double steeringAngle = -1.0472; // radians
    CuAssertDblEquals(tc, 1.065236, calculateTurnRadius(steeringAngle), .0001);

    steeringAngle = 0; // radians
    CuAssertDblEquals(tc, 1749999.999999, calculateTurnRadius(steeringAngle), .0001);

    steeringAngle = 1.04; // radians
    CuAssertDblEquals(tc, 1.081251, calculateTurnRadius(steeringAngle), .0001);
}

// --------------------------- Desired Yaw Rate Tests --------------------------------- //
void calculateDesiredYawRateTest(CuTest* tc) {
    double steeringAngle = -1;
    double velocityCG = 10;
    CuAssertDblEquals(tc, -8.523310, calculateDesiredYawRate(steeringAngle, velocityCG), .0001);

    steeringAngle = 0;
    CuAssertDblEquals(tc, -0.000006, calculateDesiredYawRate(steeringAngle, velocityCG), .0001);

    steeringAngle = 1;
    velocityCG = 35;
    CuAssertDblEquals(tc, 29.831587, calculateDesiredYawRate(steeringAngle, velocityCG), .0001);
}


// --------------------------- Calculate Yaw Error Tests --------------------------------- //
void calculateYawErrorTest(CuTest* tc) {
    double desiredYawRate = -2;
    double currentYawRate = -1.95;

    CuAssertDblEquals(tc, -0.05, calculateYawError(desiredYawRate, currentYawRate), .0001);

    desiredYawRate = 3.0;
    currentYawRate = 2.89;
    CuAssertDblEquals(tc, 0.11, calculateYawError(desiredYawRate, currentYawRate), .0001);
}

// --------------------------- Calculate Desired Yaw Moment Tests --------------------------------- //
void calculateDesiredYawMomentTest(CuTest* tc) {
    double currentYawRate = -2;
    double desiredYawRate = -1.95;
    double previousYawRate = -2.04;
    double steeringAngle = -1.5;
    double velocityCG = 25;
    double timestep = 10;

    double yawError = calculateYawError(desiredYawRate, currentYawRate);

    CuAssertDblEquals(tc, 13260.220032, calculateDesiredYawMoment(yawError, currentYawRate, desiredYawRate, previousYawRate, steeringAngle, velocityCG, timestep), .0001);

    velocityCG = 35;
    steeringAngle = 1.27;
    desiredYawRate = 1.92;
    previousYawRate = 1.75;
    currentYawRate = 1.82;
    yawError = calculateYawError(desiredYawRate, currentYawRate);
    CuAssertDblEquals(tc, 6415.159445, calculateDesiredYawMoment(yawError, currentYawRate, desiredYawRate, previousYawRate, steeringAngle, velocityCG, timestep), .0001);
}

// --------------------------- Calculate Torque Distribution Delta Tests --------------------------------- //
void calculateTorqueDistributionDeltaTest(CuTest* tc) {
    double desiredYawMoment = 13247;
    CuAssertDblEquals(tc, 519.962946, calculateTorqueDistributionDelta(desiredYawMoment), .0001);

    desiredYawMoment = 6415;
    CuAssertDblEquals(tc, 251.797562, calculateTorqueDistributionDelta(desiredYawMoment), .0001);
}

// --------------------------- Calculate Desired Torque Tests --------------------------------- //
void calculateDesiredTorqueTest(CuTest* tc) {
    double steeringAngle = -1;
    double torqueRequest = 133.5;
    double torqueDelta = 519;
    
    DesiredTorqueStruct desiredTorque;
    desiredTorque.desiredTorqueRR = 326.25;
    desiredTorque.desiredTorqueRL = -192.75;

    DesiredTorqueStruct actualDesiredTorque = calculateDesiredTorque(steeringAngle, torqueRequest, torqueDelta);

    CuAssertDblEquals(tc, desiredTorque.desiredTorqueRR, actualDesiredTorque.desiredTorqueRR, .0001);
    CuAssertDblEquals(tc, desiredTorque.desiredTorqueRL, actualDesiredTorque.desiredTorqueRL, .0001);
    
    // Test Two - different Steering Angle
    steeringAngle = 1.75;
    torqueDelta = -55;
    desiredTorque.desiredTorqueRR = 94.25;
    desiredTorque.desiredTorqueRL = 39.25;
    
    actualDesiredTorque = calculateDesiredTorque(steeringAngle, torqueRequest, torqueDelta);
    
    
    CuAssertDblEquals(tc, desiredTorque.desiredTorqueRR, actualDesiredTorque.desiredTorqueRR, .0001);
    CuAssertDblEquals(tc, desiredTorque.desiredTorqueRL, actualDesiredTorque.desiredTorqueRL, .0001);
}

// --------------------------- Calculate Vertical Load Weight Transfer Tests --------------------------------- //
void calculateVerticalLoadWeightTransferTest(CuTest* tc) {
    double accelerationLongitude = -10;
    double accelerationLatitude = -2;

    
    ForceZStruct expectedZForces;
    expectedZForces.F_zrl = 782.922343;
    expectedZForces.F_zrr = 782.769085;
    expectedZForces.F_zfl = 984.827741;
    expectedZForces.F_zfr = 984.680831;

    ForceZStruct actualZForces = calculateVerticalLoadWeightTransfer(accelerationLongitude, accelerationLatitude);

    CuAssertDblEquals(tc, expectedZForces.F_zrl, actualZForces.F_zrl, .0001);
    CuAssertDblEquals(tc, expectedZForces.F_zrr, actualZForces.F_zrr, .0001);
    CuAssertDblEquals(tc, expectedZForces.F_zfl, actualZForces.F_zfl, .0001);
    CuAssertDblEquals(tc, expectedZForces.F_zfr, actualZForces.F_zfr, .0001);

    // Test Two
    accelerationLongitude = 7;
    accelerationLatitude = 4;

    expectedZForces.F_zrl = 1254.806742;
    expectedZForces.F_zrr = 1255.113258;
    expectedZForces.F_zfl = 512.493090;
    expectedZForces.F_zfr = 512.786910;

    actualZForces = calculateVerticalLoadWeightTransfer(accelerationLongitude, accelerationLatitude);

    CuAssertDblEquals(tc, expectedZForces.F_zrl, actualZForces.F_zrl, .0001);
    CuAssertDblEquals(tc, expectedZForces.F_zrr, actualZForces.F_zrr, .0001);
    CuAssertDblEquals(tc, expectedZForces.F_zfl, actualZForces.F_zfl, .0001);
    CuAssertDblEquals(tc, expectedZForces.F_zfr, actualZForces.F_zfr, .0001);
}

// --------------------------- Calculate Lateral Forces Tests --------------------------------- //
void calculateLateralForcesTest(CuTest* tc) {
    double accelerationLatitude = -2;
    ForceZStruct ZForces;
    ZForces.F_zrl = 782.922343;
    ZForces.F_zrr = 782.769085;
    ZForces.F_zfl = 984.827741;
    ZForces.F_zfr = 984.680831;

    ForceYStruct actualYForces = calculateLateralForces(accelerationLatitude, ZForces);

    ForceYStruct expectedYForces;
    expectedYForces.F_yrl = -159.454652;
    expectedYForces.F_yrr = -159.423439;
    expectedYForces.F_yfl = -200.575915;
    expectedYForces.F_yfr = -200.545994;

    CuAssertDblEquals(tc, expectedYForces.F_yrl, actualYForces.F_yrl, .0001);
    CuAssertDblEquals(tc, expectedYForces.F_yrr, actualYForces.F_yrr, .0001);
    CuAssertDblEquals(tc, expectedYForces.F_yfl, actualYForces.F_yfl, .0001);
    CuAssertDblEquals(tc, expectedYForces.F_yfr, actualYForces.F_yfr, .0001);

    // Test Two
    accelerationLatitude = 7.3;

    ZForces.F_zrl = 1254.806742;
    ZForces.F_zrr = 1255.113258;
    ZForces.F_zfl = 512.493090;
    ZForces.F_zfr = 512.786910;

    actualYForces = calculateLateralForces(accelerationLatitude, ZForces);

    expectedYForces.F_yrl = 932.799309;
    expectedYForces.F_yrr = 933.027167;
    expectedYForces.F_yfl = 380.977552;
    expectedYForces.F_yfr = 381.195972;

    CuAssertDblEquals(tc, expectedYForces.F_yrl, actualYForces.F_yrl, .0001);
    CuAssertDblEquals(tc, expectedYForces.F_yrr, actualYForces.F_yrr, .0001);
    CuAssertDblEquals(tc, expectedYForces.F_yfl, actualYForces.F_yfl, .0001);
    CuAssertDblEquals(tc, expectedYForces.F_yfr, actualYForces.F_yfr, .0001);
}

// --------------------------- Calculate Traction Limit Torque Tests --------------------------------- //
void calculateTractionLimitTorqueTest(CuTest* tc) {
    ForceYStruct YForces;
    YForces.F_yrl = -159.454652;
    YForces.F_yrr = -159.423439;
    YForces.F_yfl = -200.575915;
    YForces.F_yfr = -200.545994;

    ForceZStruct ZForces;
    ZForces.F_zrl = 782.922343;
    ZForces.F_zrr = 782.769085;
    ZForces.F_zfl = 984.827741;
    ZForces.F_zfr = 984.680831;

    TMaxStruct expectedTMaxes;
    expectedTMaxes.maxTorqueRL = 67.918075;
    expectedTMaxes.maxTorqueRR = 67.904780;
    
    TMaxStruct tMaxes = calculateTractionLimitTorque(YForces, ZForces);

    CuAssertDblEquals(tc, expectedTMaxes.maxTorqueRL, tMaxes.maxTorqueRL, .0001);
    CuAssertDblEquals(tc, expectedTMaxes.maxTorqueRR, tMaxes.maxTorqueRR, .0001);

    // Test Two
    YForces.F_yrl = 932.799309;
    YForces.F_yrr = 933.027167;
    YForces.F_yfl = 380.977552;
    YForces.F_yfr = 381.195972;

    ZForces.F_zrl = 1254.806742;
    ZForces.F_zrr = 1255.113258;
    ZForces.F_zfl = 512.493090;
    ZForces.F_zfr = 512.786910;

    expectedTMaxes.maxTorqueRL = 108.837726;
    expectedTMaxes.maxTorqueRR = 108.864315;
    
    tMaxes = calculateTractionLimitTorque(YForces, ZForces);

    CuAssertDblEquals(tc, expectedTMaxes.maxTorqueRL, tMaxes.maxTorqueRL, .0001);
    CuAssertDblEquals(tc, expectedTMaxes.maxTorqueRR, tMaxes.maxTorqueRR, .0001);
}

// --------------------------- Traction Limit Check Tests --------------------------------- //
void tractionLimitCheckTest(CuTest* tc) {
    DesiredTorqueStruct desiredTorque;
    desiredTorque.desiredTorqueRR = 326.25;
    desiredTorque.desiredTorqueRL = -192.75;

    TMaxStruct tMaxes;
    tMaxes.maxTorqueRR = 67.904780;
    tMaxes.maxTorqueRL = 67.918075;


    NewTorqueStruct expectedNewTorques;
    expectedNewTorques.newTorqueRR = 67.904780;
    expectedNewTorques.newTorqueRL = -451.095220;

    NewTorqueStruct newTorques = tractionLimitCheck(desiredTorque, tMaxes);
    CuAssertDblEquals(tc, expectedNewTorques.newTorqueRR, newTorques.newTorqueRR, .0001);
    CuAssertDblEquals(tc, expectedNewTorques.newTorqueRL, newTorques.newTorqueRL, .0001);

    // Test Two - under max
    desiredTorque.desiredTorqueRR = 80;
    desiredTorque.desiredTorqueRL = 87;

    tMaxes.maxTorqueRR = 90;
    tMaxes.maxTorqueRL = 90;

    expectedNewTorques.newTorqueRR = 80;
    expectedNewTorques.newTorqueRL = 87;

    newTorques = tractionLimitCheck(desiredTorque, tMaxes);
    CuAssertDblEquals(tc, expectedNewTorques.newTorqueRR, newTorques.newTorqueRR, .0001);
    CuAssertDblEquals(tc, expectedNewTorques.newTorqueRL, newTorques.newTorqueRL, .0001);

    // Test Three - end rescale
    desiredTorque.desiredTorqueRR = 120;
    desiredTorque.desiredTorqueRL = 120;

    tMaxes.maxTorqueRR = 140;
    tMaxes.maxTorqueRL = 140;

    expectedNewTorques.newTorqueRR = 89;
    expectedNewTorques.newTorqueRL = 89;

    newTorques = tractionLimitCheck(desiredTorque, tMaxes);
    CuAssertDblEquals(tc, expectedNewTorques.newTorqueRR, newTorques.newTorqueRR, .0001);
    CuAssertDblEquals(tc, expectedNewTorques.newTorqueRL, newTorques.newTorqueRL, .0001);
}



CuSuite* TVGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, calculateTorqueRequestTest);
    SUITE_ADD_TEST(suite, calculateTurnRadiusTest);
    SUITE_ADD_TEST(suite, calculateDesiredYawRateTest);
    SUITE_ADD_TEST(suite, calculateYawErrorTest);
    SUITE_ADD_TEST(suite, calculateDesiredYawMomentTest);
    SUITE_ADD_TEST(suite, calculateTorqueDistributionDeltaTest);
    SUITE_ADD_TEST(suite, calculateDesiredTorqueTest);
    SUITE_ADD_TEST(suite, calculateVerticalLoadWeightTransferTest);
    SUITE_ADD_TEST(suite, calculateLateralForcesTest);
    SUITE_ADD_TEST(suite, calculateTractionLimitTorqueTest);
    SUITE_ADD_TEST(suite, tractionLimitCheckTest);

    return suite;
}

