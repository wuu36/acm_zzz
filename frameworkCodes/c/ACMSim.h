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

/* ============================================
   Test Motor Parameters (SEW 100W PMSM)
   ============================================ */
#define TEST_MOTOR_NPP  4           /* Pole pairs */
#define TEST_MOTOR_IN   3.5         /* Rated current [Arms] */
#define TEST_MOTOR_R    0.475       /* Stator resistance [Ohm] */
#define TEST_MOTOR_LD   2.05e-3     /* D-axis inductance [H] */
#define TEST_MOTOR_LQ   2.05e-3     /* Q-axis inductance [H] */
#define TEST_MOTOR_KE   0.0107      /* Back-EMF coefficient [Wb] */
#define TEST_MOTOR_RREQ 0.0         /* 0 for PMSM */
#define TEST_MOTOR_JS   0.035e-4    /* Inertia [kg.m^2] */
#define TEST_MOTOR_VDC  48          /* DC bus voltage [V] */

/* control layer parameters */
#define CL_TS       1e-4            /* control loop time steps [s] */

/*==============================================================================
    TYPES
==============================================================================*/

struct MachineSimulated {
    int npp;    /* Pole pairs */
    REAL npp_inv;           /* 1/npp for calculation efficiency */
    REAL IN;                /* Rated current [Arms] */

    /* Electrical parameters */
    REAL R;                 /* Stator resistance [Ohm] */
    REAL Ld;                /* D-axis inductance [H] */
    REAL Lq;                /* Q-axis inductance [H] */
    REAL KE;                /* Back-EMF constant / PM flux linkage [Wb] */
    REAL Rreq;              /* Rotor resistance (for IM) [Ohm] */

    /* Mechanical parameters */
    REAL Js;                /* Inertia [kg.m^2] */
    REAL Js_inv;            /* 1/Js for calculation efficiency */

    /* States */
    int NS;                 /* Number of states */
    REAL x[MACHINE_NUMBER_OF_STATES];       /* State vector */
    REAL x_dot[MACHINE_NUMBER_OF_STATES];   /* State derivative vector */
    REAL timebase;          /* Simulation time [s] */

    /* Inputs */
    REAL uAB_dist[2];       /* Disturbance voltage in alpha-beta frame */
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
    REAL iuvw[3];           /* Three-phase current [A] */
    REAL Tem;               /* Electromagnetic torque [Nm] */
    REAL cosT;              /* cos(theta_d) */
    REAL sinT;              /* sin(theta_d) */
    REAL cosT_delay_1p5omegaTs;  /* cos(theta_d - 1.5*omega*Ts) for delay compensation */
    REAL sinT_delay_1p5omegaTs;  /* sin(theta_d - 1.5*omega*Ts) */
    REAL powerfactor;       /* Power factor */

    /* Simulation settings */
    int MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;  /* Simulation steps per control period */
    REAL Ts;                /* simulation time step [s] */
    REAL current_theta;     /* current angle for transformation */
    REAL voltage_theta;     /* voltage angle for transformation */
    
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
