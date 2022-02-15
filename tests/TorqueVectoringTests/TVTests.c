#include <stdlib.h>
// #include <check.h>
#include <stdio.h>
#include <math.h>
#include "../../src/torque_vectoring/torque_vectoring.h"
#include "../CuTest.h"

void calculateTorqueRequestTest(CuTest* tc) {
    CuAssertDblEquals(tc, 2.0, 1.0, 1.5);
}

void garthTest(CuTest* tc) {
    CuAssertIntEquals(tc, 2.0, calculateTorqueRequest(), 1.4);
}


// ---------------------------Torque Request Tests--------------------------------- //

CuSuite* TVGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, calculateTorqueRequestTest);

    return suite;
}

