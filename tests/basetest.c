#include <stdlib.h>
#include <check.h>
#include "../src/torque_vectoring/torque_vectoring.h"

START_TEST(testing_helper_func)
{
    ck_assert_int_eq(helper_func(5), 5);
}
END_TEST

Suite * basetest_suite(void) {
    Suite *s;
    TCase *tc_core;

    s = suite_create("Basetestsuite");

    tc_core = tcase_create("core");

    tcase_add_test(tc_core, testing_helper_func);
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