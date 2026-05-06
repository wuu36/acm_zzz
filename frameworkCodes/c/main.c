//============================================================================
//  Includes
//============================================================================
#include "ACMSim.h"
#include <stdio.h>
#include <stdlib.h>

//============================================================================
// Local Macros
//============================================================================

/* Data file path */
#define DATA_FILE_NAME "../dat/test_motor.dat"

/* Test motor parameters (SEW 100W motor) */
#define TEST_MOTOR_NPP  4


//============================================================================
// Local Types
//============================================================================

//============================================================================
// Local Variables
//============================================================================

//============================================================================
// Public Types
//============================================================================

//============================================================================
// Public Variables
//============================================================================

struct MachineSimulated ACM;


//============================================================================
// Local Function Prototypes
//============================================================================

void init_Machine(void);

//============================================================================
// Function declarations.
//============================================================================

/* 
 * Main Entry Point
*/
int main(void) {
    printf("=== Electric Machinery Simulation ===\n");
    printf("Phase 1 MVP - Open-loop voltage test\n\n");
    
    /* Initialize motor */
    init_Machine();
    print_info();
    
    /* Open data file */
    FILE *fw = fopen(DATA_FILE_NAME, "w");
    if(fw == NULL) {
        printf("Error: Cannot open data file %s\n", DATA_FILE_NAME);
        return 1;
    }
    write_header_to_file(fw);

    /* Simulation timing */
    /* */
    /* */
    /* */
    /* Write data */
    write_data_to_file(fw);


    fclose(fw);

    printf("\n=== Simulaton Complete ===\n");
    printf("Data saved to: %s\n", DATA_FILE_NAME);
    
    return 0;
}

void init_Machine(void) {
    /* Name plate data */
    ACM.npp = TEST_MOTOR_NPP;
    
    /* Outputs initialization */
    ACM.varTheta = 0.0;
    ACM.varOmega = 0.0;
    
    /* Time */
    ACM.timebase = 0.0;
}

//-------------------- End of File -------------------------------------------







