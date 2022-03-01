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
void calcYawRateRequestTest(CuTest* tc) {
    double steeringAngle = -1;
    double velocityCG = 10;
    CuAssertDblEquals(tc, -8.523310, calcYawRateRequest(steeringAngle, velocityCG), 0.0001);

    steeringAngle = 0;
    CuAssertDblEquals(tc, -0.000006, calcYawRateRequest(steeringAngle, velocityCG), 0.0001);

    steeringAngle = 1;
    velocityCG = 35;
    CuAssertDblEquals(tc, 29.831587, calcYawRateRequest(steeringAngle, velocityCG), 0.0001);
}


// --------------------------- Yaw Error Tests --------------------------------- //
void calcYawErrorTest(CuTest* tc) {
    double requestedYawRate = -2;
    double currYawRate = -1.95;

    CuAssertDblEquals(tc, -0.05, calcYawError(requestedYawRate, currYawRate), 0.0001);

    requestedYawRate = 3.0;
    currYawRate = 2.89;
    CuAssertDblEquals(tc, 0.11, calcYawError(requestedYawRate, currYawRate), 0.0001);
}

// --------------------------- Desired Yaw Moment Tests --------------------------------- //
void calcYawMomentRequestTest(CuTest* tc) {
    double currYawRate = -2;
    double requestedYawRate = -1.95;
    double prevYawRate = -2.04;
    double steeringAngle = -1.5;
    double velocityCG = 25;
    double timestep = 10;

    double yawError = calcYawError(requestedYawRate, currYawRate);

    CuAssertDblEquals(tc, 13260.220032, calcYawMomentRequest(yawError, currYawRate, requestedYawRate, prevYawRate, steeringAngle, velocityCG, timestep), .0001);

    velocityCG = 35;
    steeringAngle = 1.27;
    requestedYawRate = 1.92;
    prevYawRate = 1.75;
    currYawRate = 1.82;
    yawError = calcYawError(requestedYawRate, currYawRate);
    CuAssertDblEquals(tc, 6415.159445, calcYawMomentRequest(yawError, currYawRate, requestedYawRate, prevYawRate, steeringAngle, velocityCG, timestep), .0001);
}

// --------------------------- Torque Distribution Delta Tests --------------------------------- //
void calcTorqueDistributionDeltaTest(CuTest* tc) {
    double requestedYawMoment = 13247;
    CuAssertDblEquals(tc, 519.962946, calcTorqueDistributionDelta(requestedYawMoment), 0.0001);

    requestedYawMoment = 6415;
    CuAssertDblEquals(tc, 251.797562, calcTorqueDistributionDelta(requestedYawMoment), 0.0001);
}

// --------------------------- Desired Torque Tests --------------------------------- //
void calcRequestedTorqueTest(CuTest* tc) {
    double steeringAngle = -1;
    double torqueRequest = 133.5;
    double torqueDelta = 519;
    
    // Requested Torque
    torque_requested_t R;
    R.torque_requested_rear_right = 326.25;
    R.torque_requested_rear_left = -192.75;

    // Actual Requested Torque
    torque_requested_t A = calcRequestedTorque(steeringAngle, torqueRequest, torqueDelta);

    CuAssertDblEquals(tc, R.torque_requested_rear_right, A.torque_requested_rear_right, 0.0001);
    CuAssertDblEquals(tc, R.torque_requested_rear_left, A.torque_requested_rear_left, 0.0001);
    
    // Test Two - different Steering Angle
    steeringAngle = 1.75;
    torqueDelta = -55;
    R.torque_requested_rear_right = 94.25;
    R.torque_requested_rear_left = 39.25;
    
    A = calcRequestedTorque(steeringAngle, torqueRequest, torqueDelta);
    
    CuAssertDblEquals(tc, R.torque_requested_rear_right, A.torque_requested_rear_right, 0.0001);
    CuAssertDblEquals(tc, R.torque_requested_rear_left, A.torque_requested_rear_left, 0.0001);
}

// --------------------------- Vertical Load Weight Transfer Tests --------------------------------- //
void calcVerticalLoadWeightTransferTest(CuTest* tc) {
    double accelerationLongitude = -10;
    double accelerationLatitude = -2;

    
    force_z_t expected_Z;
    expected_Z.force_z_rear_left = 782.922343;
    expected_Z.force_z_rear_right = 782.769085;
    expected_Z.force_z_front_left = 984.827741;
    expected_Z.force_z_front_right = 984.680831;

    force_z_t actual_Z = calcVerticalLoadWeightTransfer(accelerationLongitude, accelerationLatitude);

    CuAssertDblEquals(tc, expected_Z.force_z_rear_left, actual_Z.force_z_rear_left, 0.0001);
    CuAssertDblEquals(tc, expected_Z.force_z_rear_right, actual_Z.force_z_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_Z.force_z_front_left, actual_Z.force_z_front_left, 0.0001);
    CuAssertDblEquals(tc, expected_Z.force_z_front_right, actual_Z.force_z_front_right, 0.0001);

    // Test Two
    accelerationLongitude = 7;
    accelerationLatitude = 4;

    expected_Z.force_z_rear_left = 1254.806742;
    expected_Z.force_z_rear_right = 1255.113258;
    expected_Z.force_z_front_left = 512.493090;
    expected_Z.force_z_front_right = 512.786910;

    actual_Z = calcVerticalLoadWeightTransfer(accelerationLongitude, accelerationLatitude);

    CuAssertDblEquals(tc, expected_Z.force_z_rear_left, actual_Z.force_z_rear_left, 0.0001);
    CuAssertDblEquals(tc, expected_Z.force_z_rear_right, actual_Z.force_z_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_Z.force_z_front_left, actual_Z.force_z_front_left, 0.0001);
    CuAssertDblEquals(tc, expected_Z.force_z_front_right, actual_Z.force_z_front_right, 0.0001);
}

// --------------------------- Lateral Forces Tests --------------------------------- //
void calcLateralForcesTest(CuTest* tc) {
    double accelerationLatitude = -2;
    force_z_t Z;
    Z.force_z_rear_left = 782.922343;
    Z.force_z_rear_right = 782.769085;
    Z.force_z_front_left = 984.827741;
    Z.force_z_front_right = 984.680831;

    force_y_t actual_Y = calcLateralForces(accelerationLatitude, Z);

    force_y_t expected_Y;
    expected_Y.force_y_rear_left = -159.454652;
    expected_Y.force_y_rear_right = -159.423439;
    expected_Y.force_y_front_left = -200.575915;
    expected_Y.force_y_front_right = -200.545994;

    CuAssertDblEquals(tc, expected_Y.force_y_rear_left, actual_Y.force_y_rear_left, 0.0001);
    CuAssertDblEquals(tc, expected_Y.force_y_rear_right, actual_Y.force_y_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_Y.force_y_front_left, actual_Y.force_y_front_left, 0.0001);
    CuAssertDblEquals(tc, expected_Y.force_y_front_right, actual_Y.force_y_front_right, 0.0001);

    // Test Two
    accelerationLatitude = 7.3;

    Z.force_z_rear_left = 1254.806742;
    Z.force_z_rear_right = 1255.113258;
    Z.force_z_front_left = 512.493090;
    Z.force_z_front_right = 512.786910;

    actual_Y = calcLateralForces(accelerationLatitude, Z);

    expected_Y.force_y_rear_left = 932.799309;
    expected_Y.force_y_rear_right = 933.027167;
    expected_Y.force_y_front_left = 380.977552;
    expected_Y.force_y_front_right = 381.195972;

    CuAssertDblEquals(tc, expected_Y.force_y_rear_left, actual_Y.force_y_rear_left, 0.0001);
    CuAssertDblEquals(tc, expected_Y.force_y_rear_right, actual_Y.force_y_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_Y.force_y_front_left, actual_Y.force_y_front_left, 0.0001);
    CuAssertDblEquals(tc, expected_Y.force_y_front_right, actual_Y.force_y_front_right, 0.0001);
}

// --------------------------- Traction Limit Torque Tests --------------------------------- //
void calcTractionLimitTorqueTest(CuTest* tc) {
    force_y_t Y;
    Y.force_y_rear_left = -159.454652;
    Y.force_y_rear_right = -159.423439;
    Y.force_y_front_left = -200.575915;
    Y.force_y_front_right = -200.545994;

    force_z_t Z;
    Z.force_z_rear_left = 782.922343;
    Z.force_z_rear_right = 782.769085;
    Z.force_z_front_left = 984.827741;
    Z.force_z_front_right = 984.680831;

    torque_max_t expected_TMax;
    expected_TMax.torque_max_rear_left = 67.918075;
    expected_TMax.torque_max_rear_right = 67.904780;
    
    torque_max_t actual_TMax = calcTractionLimitTorque(Y, Z);

    CuAssertDblEquals(tc, expected_TMax.torque_max_rear_left, actual_TMax.torque_max_rear_left, 0.0001);
    CuAssertDblEquals(tc, expected_TMax.torque_max_rear_right, actual_TMax.torque_max_rear_right, 0.0001);

    // Test Two
    Y.force_y_rear_left = 932.799309;
    Y.force_y_rear_right = 933.027167;
    Y.force_y_front_left = 380.977552;
    Y.force_y_front_right = 381.195972;

    Z.force_z_rear_left = 1254.806742;
    Z.force_z_rear_right = 1255.113258;
    Z.force_z_front_left = 512.493090;
    Z.force_z_front_right = 512.786910;

    expected_TMax.torque_max_rear_left = 108.837726;
    expected_TMax.torque_max_rear_right = 108.864315;
    
    actual_TMax = calcTractionLimitTorque(Y, Z);

    CuAssertDblEquals(tc, expected_TMax.torque_max_rear_left, actual_TMax.torque_max_rear_left, 0.0001);
    CuAssertDblEquals(tc, expected_TMax.torque_max_rear_right, actual_TMax.torque_max_rear_right, 0.0001);
}

// --------------------------- Traction Limit Check Tests --------------------------------- //
void checkTractionLimitTest(CuTest* tc) {
    torque_requested_t R;
    R.torque_requested_rear_right = 326.25;
    R.torque_requested_rear_left = -192.75;

    torque_max_t M;
    M.torque_max_rear_right = 67.904780;
    M.torque_max_rear_left = 67.918075;

    torque_corrected_t expected_newT;
    expected_newT.torque_corrected_rear_right = 67.904780;
    expected_newT.torque_corrected_rear_left = -451.095220;

    torque_corrected_t actual_newT = checkTractionLimit(R, M);
    CuAssertDblEquals(tc, expected_newT.torque_corrected_rear_right, actual_newT.torque_corrected_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_newT.torque_corrected_rear_left, actual_newT.torque_corrected_rear_left, 0.0001);

    // Test Two - under max
    R.torque_requested_rear_right = 80;
    R.torque_requested_rear_left = 87;

    M.torque_max_rear_right = 90;
    M.torque_max_rear_left = 90;

    expected_newT.torque_corrected_rear_right = 80;
    expected_newT.torque_corrected_rear_left = 87;

    actual_newT = checkTractionLimit(R, M);
    CuAssertDblEquals(tc, expected_newT.torque_corrected_rear_right, actual_newT.torque_corrected_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_newT.torque_corrected_rear_left, actual_newT.torque_corrected_rear_left, 0.0001);

    // Test Three - end rescale
    R.torque_requested_rear_right = 120;
    R.torque_requested_rear_left = 120;

    M.torque_max_rear_right = 140;
    M.torque_max_rear_left = 140;

    expected_newT.torque_corrected_rear_right = 89;
    expected_newT.torque_corrected_rear_left = 89;

    actual_newT = checkTractionLimit(R, M);
    CuAssertDblEquals(tc, expected_newT.torque_corrected_rear_right, actual_newT.torque_corrected_rear_right, 0.0001);
    CuAssertDblEquals(tc, expected_newT.torque_corrected_rear_left, actual_newT.torque_corrected_rear_left, 0.0001);
}


CuSuite* TorqueVectoringGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, calcTorqueRequestTest);
    SUITE_ADD_TEST(suite, calcTurnRadiusTest);
    SUITE_ADD_TEST(suite, calcYawRateRequestTest);
    SUITE_ADD_TEST(suite, calcYawErrorTest);
    SUITE_ADD_TEST(suite, calcYawMomentRequestTest);
    SUITE_ADD_TEST(suite, calcTorqueDistributionDeltaTest);
    SUITE_ADD_TEST(suite, calcRequestedTorqueTest);
    SUITE_ADD_TEST(suite, calcVerticalLoadWeightTransferTest);
    SUITE_ADD_TEST(suite, calcLateralForcesTest);
    SUITE_ADD_TEST(suite, calcTractionLimitTorqueTest);
    SUITE_ADD_TEST(suite, checkTractionLimitTest);

    return suite;
}