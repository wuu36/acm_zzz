#ifndef _ACMSIM_H
#define _ACMSIM_H

/*==============================================================================
    INCLUDES
==============================================================================*/

#include <stdio.h> /* printf FILE */

#include <math.h>
#include <time.h>


/* header files in one place */
/* everything that is configurable is in here*/
#include "super_config.h"


#include "main_switch.h"

/*==============================================================================
    DEFINES
==============================================================================*/

/* main simulation */
#define MACHINE_NUMBER_OF_STATES 5


/* this is not experiment */
#define PC_SIMULATION TRUE

#if PC_SIMULATION
    #define __INVERTER_NONLINEARITY 0
#endif

/*==============================================================================
    TYPES
==============================================================================*/

struct MachineSimulated {
    // # name plate data
    int npp;
    REAL npp_inv;


    // # states
    REAL x[MACHINE_NUMBER_OF_STATES];
    REAL timebase;

    // # inputs
    REAL uAB_inverter[2];
    REAL uAB[2];
    REAL uDQ[2];

    // # outputs

    REAL omega_syn;

    REAL theta_d;

    REAL cosT;
    REAL sinT;
    REAL cosT_delay_1p5omegaTs;
    REAL sinT_delay_1p5omegaTs;


    // # simulation settings
    REAL Ts;
};


/*==============================================================================
    GLOBAL VARIABLE DECLARATIONS
==============================================================================*/


/*==============================================================================
    INLINE FUNCTION DEFINITIONS
==============================================================================*/


/*==============================================================================
    FUNCTION DECLARATIONS
==============================================================================*/

void init_Machine();
int machine_simulation();
void inverter_model();
void measurement();

/* declaration of utility function defined in utility.c */

void write_header_to_file(FILE *fw);
void print_info();


#endif /* _ACMSIM_H */
/*-------------------- End of File -------------------------------------------*/
