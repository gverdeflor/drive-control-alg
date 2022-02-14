#include <stdlib.h>
#include <check.h>
#include <stdio.h>
#include <math.h>
#include "../../src/torque_vectoring/torque_vectoring.h"

#define 	ck_assert_float_eq(X, Y)   _ck_assert_floating(X, ==, Y, float, "")

START_TEST(calculateTorqueRequestTest)
{   
    printf("BENIS\n\n");
    printf("%f\n\n", calculateTorqueRequest(.2044));
    ck_assert_float_eq(calculateTorqueRequest(.2044), 36.794);
}
END_TEST


Suite * basetest_suite(void) {
    Suite *s;
    TCase *tc_core;

    s = suite_create("TV test suite");

    tc_core = tcase_create("TV");

    tcase_add_test(tc_core, calculateTorqueRequestTest);
    suite_add_tcase(s, tc_core);

    return s;
}


int main(void) {
    // return 0;
    Suite *s;
    SRunner *sr;
    int number_failed;

    s = basetest_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);

    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}