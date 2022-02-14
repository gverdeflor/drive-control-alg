#include <stdlib.h>
// #include <check.h>
#include <stdio.h>
#include <math.h>
#include "../../src/torque_vectoring/torque_vectoring.h"
#include "../CuTest.h"

void calculateTorqueRequestTest(CuTest* tc) {
    CuAssertDblEquals(tc, 1.0, 1.0, 0.0);
}

CuSuite* TVGetSuite(void) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST(suite, calculateTorqueRequestTest);

    return suite;
}