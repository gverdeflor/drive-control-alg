/* test_battery_state_all.c ---
 * 
 *
 * Author: Suraj Srivats, Garth Verdeflor
 * Created: Tues Feb 15 11:33:29 2022 (-0400)
 * Version: 1.0  
 *
 *
 * Description: Tests all battery state algorithm calculations.
 *
 */

#include <stdio.h>
#include "../CuTest.h"
    
CuSuite* BatteryStateGetSuite();

void RunAllTests(void) {
    CuString *output = CuStringNew();
    CuSuite* suite = CuSuiteNew();
    
    CuSuiteAddSuite(suite, BatteryStateGetSuite());

    CuSuiteRun(suite);
    CuSuiteSummary(suite, output);
    CuSuiteDetails(suite, output);
    printf("%s\n", output->buffer);
}

int main(void) {
    RunAllTests();
}