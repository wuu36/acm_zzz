//============================================================================
//  Includes
//============================================================================
#include "ACMSim.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

//============================================================================
// Local Macros
//============================================================================


/* Simulation parameters */
#define CL_TS   1e-4
#define MACHINE_TS  (CL_TS / 1.0)
#define NUMBER_OF_STEPS 50000

/* Data file path */
#define DATA_FILE_NAME "../dat/test_motor.dat"

/* Test motor parameters (SEW 100W motor) */
#define TEST_MOTOR_NPP  4

#define TEST_MOTOR_R    0.475   /* Ohm */
#define TEST_MOTOR_LD   2.05e-3 /* H = 2.05 mH */
#define TEST_MOTOR_LQ   2.05e-3 /* H*/
#define TEST_MOTOR_KE   0.0107  /* Wb */
#define TEST_MOTOR_RREQ 0.0     /* 0 for PMSM */
#define TEST_MOTOR_JS   0.035e-4    /* kg.mm^2 = 0.035 kg.cm^2 */


/* Test vlotage command */
#define TEST_VOLTAGE_UD 0.0     /* V */
#define TEST_VOLTAGE_UQ 5.0     /* V */


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


void measurement(void);
void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]);
void RK4(REAL t, REAL *x, REAL hs);
void inverter_modle(void);
void init_Machine(void);
int machine_simulation(void);

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
    clock_t begin = clock();

    /* Set initial voltage command in D-Q frame */
    ACM.uDQ[0] = TEST_VOLTAGE_UD;
    ACM.uDQ[1] = TEST_VOLTAGE_UQ;
    
    /* Initial Park transform to get alpha-beta voltage */
    ACM.theta_d = ACM.x[0] * ACM.npp;
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);
    ACM.uAB[0] = MT2A(ACM.uDQ[0], ACM.uDQ[1], ACM.cosT, ACM.sinT);
    ACM.uAB[1] = MT2B(ACM.uDQ[0], ACM.uDQ[1], ACM.cosT, ACM.sinT);

    /* Main simulation loop */
    int step;
    for (step = 0; step < NUMBER_OF_STEPS; step++) {
        ACM.timebase = step * CL_TS;
        
        /* update voltage angle for open-loop test */
        ACM.theta_d = ACM.x[0] * ACM.npp;
        ACM.cosT = cos(ACM.theta_d);
        ACM.sinT = sin(ACM.theta_d);

        /* keep uDQ constant, transform to uAB */
        ACM.uAB[0] = MT2A(TEST_VOLTAGE_UD, TEST_VOLTAGE_UQ, ACM.cosT, ACM.sinT);
        ACM.uAB[1] = MT2B(TEST_VOLTAGE_UD, TEST_VOLTAGE_UQ, ACM.cosT, ACM.sinT);
        

        /* run one simulation step */
        if (machine_simulation()) {
            printf("Simulation stopped at step %d\n", step);
            break;
        }

        /* Write data */
        measurement();
        write_data_to_file(fw);
        
        /* progress indicator */
        if (step % 10000 == 0) {
            printf("Step %d / %d (%.1f%%), Omega = %.2f rad/s, iQ = %.3f A\n", 
                    step, NUMBER_OF_STEPS, 100.0 * step / NUMBER_OF_STEPS,
                    ACM.varOmega, ACM.iDQ[1]);
        }
    }

    /* Cleanup */
    clock_t end = clock();
    fclose(fw);

    printf("\n=== Simulaton Complete ===\n");
    printf("Total steps: %d\n", step);
    printf("Final time: %.4f s\n", ACM.timebase);
    printf("Final speed: %.2f rad/s (%.2f rpm)\n", ACM.varOmega, ACM.varOmega * MECH_RAD_PER_SEC_2_RPM);
    printf("Final current: iD=%.3f A, iQ=%.3f A\n", ACM.iDQ[0], ACM.iDQ[1]);
    printf("Final torque: %.4f Nm\n", ACM.Tem);
    printf("Simulation time: %.3f seconds\n", (REAL)(end - begin) / CLOCKS_PER_SEC);
    printf("Data saved to: %s\n", DATA_FILE_NAME);
    
    return 0;
}

void init_Machine(void) {
    /* Name plate data */
    ACM.npp = TEST_MOTOR_NPP;

    /* Electrical parameters */
    ACM.R = TEST_MOTOR_R;
    ACM.Ld = TEST_MOTOR_LD;
    ACM.Lq = TEST_MOTOR_LQ;
    ACM.KE = TEST_MOTOR_KE;
    ACM.KA = ACM.KE;
    ACM.Rreq = TEST_MOTOR_RREQ;

    /* Mechanical parameters */
    ACM.Js = TEST_MOTOR_JS;

    /* States initialization */
    ACM.NS = MACHINE_NUMBER_OF_STATES;
    for (int i = 0; i < ACM.NS; i++) {
        ACM.x[i] = 0.0;
        ACM.x_dot[i] = 0.0;
    }

    /* for PMSM, KA (PM flux) is constant, initialize x[2] */
    if (ACM.Rreq <= 0) {
        ACM.x[2] = ACM.KE;
    }


    /* Inputs initialization */
    
    ACM.uAB_inverter[0] = 0.0;
    ACM.uAB_inverter[1] = 0.0;
    ACM.uAB[0] = 0.0;
    ACM.uAB[1] = 0.0;
    ACM.uDQ[0] = TEST_VOLTAGE_UD;
    ACM.uDQ[1] = TEST_VOLTAGE_UQ;
    ACM.TLoad = 0.0;
    
    /* Outputs initialization */
    ACM.varTheta = 0.0;
    ACM.varOmega = 0.0;
    ACM.omega_syn = 0.0;
    ACM.omega_slip = 0.0;
    ACM.theta_d = 0.0;
    ACM.iDQ[0] = 0.0;
    ACM.iDQ[1] = 0.0;
    ACM.iAB[0] = 0.0;
    ACM.iAB[1] = 0.0;
    ACM.psi_AB[0] = 0.0;
    ACM.psi_AB[1] = 0.0;

    ACM.Tem = 0.0;
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);


    /* Simulation settings */
    
    ACM.Ts = MACHINE_TS;
    
    /* Time */
    ACM.timebase = 0.0;
}

void inverter_modle(void) {
    /* for MVP, use ideal inverter (no nonlinearity) */
    ACM.uAB_inverter[0] = ACM.uAB[0];
    ACM.uAB_inverter[1] = ACM.uAB[1];
}

int machine_simulation(void) {
    /* apply inverter model */
    inverter_modle();

    /* transform voltage from alpha-beta to D-Q (Park transform) */
    ACM.uDQ[0] = AB2M(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
    ACM.uDQ[1] = AB2T(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
    
    /* numerical integration using RK4 */
    RK4(ACM.timebase, ACM.x, ACM.Ts);

    /* update output variables from states */
    ACM.varOmega = ACM.x[1];    /* mechanical speed */
    ACM.varTheta = ACM.x[0];    /* mechanical position */
    ACM.theta_d = ACM.varTheta * ACM.npp;    /* electrical position */
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);

    /* current in D-Q frame */
    ACM.iDQ[0] = ACM.x[3];
    ACM.iDQ[1] = ACM.x[4];

    /* current in alpha-beta frame (inverse Park transform) */
    ACM.iAB[0] = MT2A(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);
    ACM.iAB[1] = MT2B(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);

    /* flux linkage */
    ACM.KA = ACM.x[2];
    ACM.psi_AB[0] = ACM.KA * ACM.cosT;
    ACM.psi_AB[1] = ACM.KA * ACM.sinT;

    /* Back-EMF */
    ACM.emf_AB[0] = ACM.x_dot[2] * ACM.cosT + ACM.KA * (-sin(ACM.theta_d)) * (ACM.npp * ACM.x_dot[0]);
    ACM.emf_AB[1] = ACM.x_dot[2] * ACM.sinT + ACM.KA * cos(ACM.theta_d) * (ACM.npp * ACM.x_dot[0]);
    
    /* Limit theta_d to [-pi, pi] */
    while (ACM.theta_d > M_PI)  ACM.theta_d -= 2 * M_PI;
    while (ACM.theta_d < -M_PI) ACM.theta_d += 2 * M_PI;

    /* check for simulation crash (infinite speed) */
    if (!isNumber(ACM.varOmega)) {
        printf("Warning: varOmega is NaN of Inf, stopping simulation\n");
        return TRUE;
    }

    return FALSE;   /* continue simulation */
}

/*
 * RK4 Numerical Integration
 *
 * Standard 4th-order Runge-Kutta method:
 * k1 = f(t, x) * h
 * k2 = f(t + h/2, x + k1/2) * h
 * k3 = f(t + h/2, x + k2/2) * h
 * k4 = f(t + h, x + k3) * h
 * x_new = x + (k1 + 2*k2 + 2*k3 + k4) / 6
 */
void RK4(REAL t, REAL *x, REAL hs) {
    #define NS MACHINE_NUMBER_OF_STATES

    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];

    /* k1 */
    DYNAMICS_MACHINE(t, x, fx);
    for (int i = 0; i < NS; i++) {
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i] * 0.5;
    }

    /* k2 */
    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < NS; i++) {
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i] * 0.5;
    }

    /* k3 */
    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < NS; i++) {
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }

    /* k4 */
    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < NS; i++) {
        k4[i] = fx[i] * hs;
        /* Update state */
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i]) * one_over_six;
        /* Store derivative for debugging */
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i]) * one_over_six / hs;
    }

    #undef NS
}

/*
 * Motor Dynamics - State Space Model
 *
 * States:
 *   x[0] = varTheta (mechanical position)
 *   x[1] = varOmega (mechanical speed)
 *   x[2] = KA (active flux)
 *   x[3] = iD (d-axis current)
 *   x[4] = iQ (q-axis current)
 *
 * For PMSM (Rreq = 0):
 *   KA = KE (constant)
 *   omega_slip = 0
 *   omega_syn = omega_r = x[1] * npp
 *
 * For IM (Rreq > 0):
 *   KA varies
 *   omega_slip = Rreq * iQ / KA
 *   omega_syn = omega_r + omega_slip
 */
void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]) {
    /* Extract states */
    REAL KA = x[2];
    REAL iD = x[3];
    REAL iQ = x[4];

    /* Calculate slip speed */
    if (KA == 0.0) {
        ACM.omega_slip = 0.0;
    } else {
        ACM.omega_slip = ACM.Rreq * iQ / KA;
    }

    /* Synchronous speed (electrical) */
    ACM.omega_syn = x[1] * ACM.npp + ACM.omega_slip;

    /* Electrical subsystem dynamics */
    if (ACM.Rreq > 0) {
        /* Induction Motor model */
        /* d(KA)/dt = Rreq*iD - Rreq/(Ld-Lq)*KA */
        fx[2] = ACM.Rreq * iD - ACM.Rreq / (ACM.Ld - ACM.Lq) * KA;
        /* d(iD)/dt */
        fx[3] = (ACM.uDQ[0] - ACM.R * iD + ACM.omega_syn * ACM.Lq * iQ - fx[2]) / ACM.Lq;
    } else if (ACM.Rreq < 0) {
        printf("Error: Rreq must be >= 0 for valid simulation\n");
        fx[2] = 0;
        fx[3] = 0;
    } else {
        /* PMSM model: KA = KE (constant), so d(KA)/dt = 0 */
        fx[2] = 0;
        /* d(iD)/dt = (uD - R*iD + omega*Lq*iQ) / Ld */
        fx[3] = (ACM.uDQ[0] - ACM.R * iD + ACM.omega_syn * ACM.Lq * iQ) / ACM.Ld;
    }

    /* d(iQ)/dt = (uQ - R*iQ - omega*Ld*iD - omega*KA) / Lq */
    fx[4] = (ACM.uDQ[1] - ACM.R * iQ - ACM.omega_syn * ACM.Ld * iD - ACM.omega_syn * KA) / ACM.Lq;

    /* Mechanical subsystem dynamics */
    /* Electromagnetic torque: Tem = 1.5 * npp * KA * iQ */
    ACM.Tem = CLARKE_TRANS_TORQUE_GAIN * ACM.npp * KA * iQ;

    /* d(varTheta)/dt = varOmega + omega_slip/npp */
    fx[0] = x[1] + ACM.omega_slip / ACM.npp;

    /* d(varOmega)/dt = (Tem - TLoad) / Js */
    fx[1] = (ACM.Tem - ACM.TLoad) / ACM.Js;
}

void measurement(void) {
    
}


//-------------------- End of File -------------------------------------------







