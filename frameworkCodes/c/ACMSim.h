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

    // # electrical parameters
    REAL R;
    REAL Ld;
    REAL Lq;
    REAL KE;
    REAL Rreq;

    // # mechanical parameters
    REAL Js;
    REAL Js_inv;


    // # states
    int NS;
    REAL x[MACHINE_NUMBER_OF_STATES];
    REAL x_dot[MACHINE_NUMBER_OF_STATES];
    REAL timebase;

    // # inputs
    REAL uAB_inverter[2];
    REAL uAB[2];
    REAL uDQ[2];
    REAL TLoad;

    // # outputs
    REAL varTheta;
    REAL varOmega;
    REAL omega_syn;
    REAL omega_slip;
    REAL theta_d;
    REAL KA;
    REAL iDQ[2];
    REAL iAB[2];
    
    REAL psi_AB[2];
    REAL emf_AB[2];

    REAL Tem;
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

extern struct MachineSimulated ACM;

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
void RK4(REAL t, REAL *x, REAL hs);
void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]);

/* declaration of utility function defined in utility.c */
void write_data_to_file(FILE *fw);
void write_header_to_file(FILE *fw);
void print_info();
int isNumber(REAL x);


#endif /* _ACMSIM_H */
/*-------------------- End of File -------------------------------------------*/
