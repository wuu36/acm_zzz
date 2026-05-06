//============================================================================
//  Includes
//============================================================================
# include "ACMSim.h"
#include <stdio.h>


//============================================================================
// Local Macros
//============================================================================

#define CL_TS 1e-4
#define MACHINE_TS  (CL_TS / 1.0)
#define NUMBER_OF_STEPS 50000

/* Data output configuration */
#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "time,varTheta,varOmega,KA,iD,iQ,theta_d,uAB0,uAB1,uD0,uQ1,iAB0,iAB1,TLoad,Tem\n"


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

//============================================================================
// Local Function Prototypes
//============================================================================

//============================================================================
// Function declarations.
//============================================================================



/* check if a number is valid (not NaN or Inf) */
int isNumber(REAL x) {
    return (x==x);
}


/* Write header (variable anmes ) to data file */
void write_header_to_file(FILE *fw) {
    fprintf(fw, DATA_LABELS);
}

/* Write data row to file */
void write_data_to_file(FILE *fw) {
    fprintf(fw, DATA_FORMAT,
        ACM.timebase,
        ACM.varTheta,
        ACM.varOmega,
        ACM.KA,
        ACM.iDQ[0],
        ACM.iDQ[1],
        ACM.theta_d / M_PI * 180,
        ACM.uAB[0],
        ACM.uAB[1],
        ACM.uDQ[0],
        ACM.uDQ[1],
        ACM.iAB[0],
        ACM.iAB[1],
        ACM.TLoad,
        ACM.Tem
    );
}

/* Print simulation info */
void print_info(void) {
    printf("\n");
    printf("=== Motor Simulation Configuration ===\n");
    printf("Pole pairs (npp):   %d\n", ACM.npp);
    printf("Rated current (IN):   %g Arms\n", ACM.IN);
    printf("Stator resistance (R): %g Ohm\n", ACM.R);
    printf("D-axis inductance (Ld): %g H (%g mH)\n", ACM.Ld, ACM.Ld * 1000);
    printf("Q-axis inductance (Lq): %g H (%g mH)\n", ACM.Lq, ACM.Lq * 1000);
    printf("Back-EMF constant (KE): %g Wb\n", ACM.KE);
    printf("Rotor resistance (Rreq): %g Ohm\n", ACM.Rreq);
    printf("Inertia (Js):         %g kg.m^2\n", ACM.Js);
    printf("\n");
    printf("=== Simulation Parameters ===\n");
    printf("Control time step (CL_TS): %g s (%g Hz)\n", CL_TS, 1.0/CL_TS);
    printf("Machine time step (MACHINE_TS): %g s\n", MACHINE_TS);
    printf("Total steps:          %d\n", NUMBER_OF_STEPS);
    printf("Simulation duration:  %g s\n", NUMBER_OF_STEPS * CL_TS);
    printf("\n");
}


//-------------------- End of File -------------------------------------------
