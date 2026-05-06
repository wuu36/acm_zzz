#ifndef _ACMSIM_H
#define _ACMSIM_H

/*==============================================================================
    INCLUDES
==============================================================================*/

#include <stdio.h>
#include "typedef.h"

/*==============================================================================
    DEFINES
==============================================================================*/

/* number of state in the unified motor model */
#define MACHINE_NUMBER_OF_STATES 5

/*==============================================================================
    TYPES
==============================================================================*/

struct MachineSimulated {
    int npp;    /* Pole pairs */

    REAL IN;                /* Rated current [Arms] */

    /* Electrical parameters */
    REAL R;                 /* Stator resistance [Ohm] */
    REAL Ld;                /* D-axis inductance [H] */
    REAL Lq;                /* Q-axis inductance [H] */
    REAL KE;                /* Back-EMF constant / PM flux linkage [Wb] */
    REAL Rreq;              /* Rotor resistance (for IM) [Ohm] */

    /* Mechanical parameters */
    REAL Js;                /* Inertia [kg.m^2] */

    /* States */
    int NS;                 /* Number of states */
    REAL x[MACHINE_NUMBER_OF_STATES];       /* State vector */
    REAL x_dot[MACHINE_NUMBER_OF_STATES];   /* State derivative vector */
    REAL timebase;          /* Simulation time [s] */

    /* Inputs */
    REAL uAB_inverter[2];   /* Inverter output voltage */
    REAL uAB[2];            /* Applied voltage in alpha-beta frame */
    REAL uDQ[2];            /* Applied voltage in D-Q frame */
    REAL TLoad;             /* Load torque [Nm] */

    /* Outputs */
    REAL varTheta;          /* Mechanical rotor position [rad] */
    REAL varOmega;          /* Mechanical rotor speed [rad/s] */
    REAL omega_syn;         /* Synchronous speed [elec rad/s] */
    REAL omega_slip;        /* Slip speed [elec rad/s] */
    REAL theta_d;           /* Electrical rotor position [elec rad] */
    REAL KA;                /* Active flux [Wb] */
    REAL iDQ[2];            /* Current in D-Q frame [A] */
    REAL iAB[2];            /* Crueent in alpha-beta frame [A] */
    REAL psi_AB[2];         /* Flux linkage in alpha-beta frame [Wb] */
    REAL emf_AB[2];         /* Back-EMF in alpha-beta frame [V] */


    

    REAL Tem;               /* Electromagnetic torque [Nm] */
    REAL cosT;              /* cos(theta_d) */
    REAL sinT;              /* sin(theta_d) */


    /* Simulation settings */
    REAL Ts;                /* simulation time step [s] */
    
    
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


/* utility function declarations */
int isNumber(REAL x);

void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
void print_info(void);


#endif /* _ACMSIM_H */
/*-------------------- End of File -------------------------------------------*/
