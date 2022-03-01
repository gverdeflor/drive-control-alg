/* test_torque_vectoring_all.c ---
 * 
 *
 * Author: Suraj Srivats, Garth Verdeflor
 * Created: Sun Feb 20 11:33:29 2022 (-0400)
 * Version: 1.0  
 *
 *
 * Description: Tests all torque vectoring algorithm calculations.
 *
 */

#include <stdio.h>
#include "../CuTest.h"
    
CuSuite* TorqueVectoringGetSuite();

void RunAllTests(void) {
    CuString *output = CuStringNew();
    CuSuite* suite = CuSuiteNew();
    
    CuSuiteAddSuite(suite, TorqueVectoringGetSuite());

    CuSuiteRun(suite);
    CuSuiteSummary(suite, output);
    CuSuiteDetails(suite, output);
    printf("%s\n", output->buffer);
}

int main(void) {
    RunAllTests();
}