//============================================================================
//  Includes
//============================================================================

#include <stdio.h>
#include "ACMSim.h"
#include "main_switch.h"
#include <math.h>
#include <time.h>

//============================================================================
// Local Macros
//============================================================================

/* test conditions */
#define TEST_VOLTAGE_UD 0.0
#define TEST_VOLTAGE_UQ 0.0
#define CMD_ID 0.0
#define CMD_IQ 1.0

/* simulation parameters */
#define MACHINE_TS  (CL_TS / 1.0)
#define NUMBER_OF_STEPS 10000   /* 1s simulation time */
#define MACHINE_SIM_PER_CONTROL 1

/* data file path */
#define DATA_FILE_NAME "../dat/test_motor_foc.dat"

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
int machine_simulation(void);
void inverter_model(void);
void RK4(REAL t, REAL *x, REAL hs);
void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]);

//============================================================================
// Function declarations.
//============================================================================

int main(void) {
    printf("=== Electric Machinery Simulation - FOC Test ===\n");
    printf("Testing FOC current loop:\n");
    printf("  cmd_iD = %.1f A\n", CMD_ID);
    printf("  cmd_iQ = %.1f A\n", CMD_IQ);
    printf("\n");

    /* initialize motor and controller */
    init_Machine();
    init_CTRL();

    printf("Motor parameters:\n");
    printf("  npp = %d, R = %.3f Ohm, L = %.2f mH\n", ACM.npp, ACM.R, ACM.Lq*1000);
    printf("  KE = %.4f Wb, Js = %.2e kg.m^2\n", ACM.KE, ACM.Js);
    printf("\nPI parameters:\n");
    printf("  Kp_d = %.2f, Ki_d = %.4f\n", PID_iD.Kp, PID_iD.Ki);
    printf("  Kp_q = %.2f, Ki_q = %.4f\n", PID_iQ.Kp, PID_iQ.Ki);
    printf("\nSimulation duration: %.1f s\n\n", NUMBER_OF_STEPS * CL_TS);

    /* open data file */
    FILE *fw = fopen(DATA_FILE_NAME, "w");
    if (fw == NULL) {
        printf("Error: Cannot open data file %s\n", DATA_FILE_NAME);
        return 1;
    }

    /* Write header */
    fprintf(fw, "time,varTheta,varOmega,iD,iQ,uD,uQ,cmd_iD,cmd_iQ,Tem,theta_d\n");
    
    clock_t begin = clock();

    /* initial voltage setup (zero) */
    ACM.uAB[0] = 0.0;
    ACM.uAB[1] = 0.0;

    /* main simulation loop */
    int step;

    for (step = 0; step < NUMBER_OF_STEPS; step++) {
        ACM.timebase = step * CL_TS;

        /* set current commands */
        CTRL_inputs.cmd_iDQ[0] = CMD_ID;
        CTRL_inputs.cmd_iDQ[1] = CMD_IQ;

        /* update controller inputs from motor */
        CTRL_inputs.theta_d_elec = ACM.theta_d;
        CTRL_inputs.varOmega = ACM.varOmega;
        CTRL_inputs.iAB[0] = ACM.iAB[0];
        CTRL_inputs.iAB[1] = ACM.iAB[1];

        /* run FOC control */
        main_switch(MODE_SELECT_FOC);

        /* apply controller output to motor */
        ACM.uAB[0] = CTRL_outputs.cmd_uAB[0];
        ACM.uAB[1] = CTRL_outputs.cmd_uAB[1];

        if (machine_simulation()) {
            printf("simulation stopped at step %d\n", step);
            break;
        }

        /* Write data */
        fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
            ACM.timebase,
            ACM.varTheta,
            ACM.varOmega,
            ACM.iDQ[0],
            ACM.iDQ[1],
            CTRL_outputs.cmd_uDQ[0],
            CTRL_outputs.cmd_uDQ[1],
            CTRL_inputs.cmd_iDQ[0],
            CTRL_inputs.cmd_iDQ[1],
            ACM.Tem,
            ACM.theta_d / M_PI * 180
        );

        /* Progress indicator */
        if (step % 1000 == 0) {
            printf("Step %d (%.1f%%): iD=%.3f A, iQ=%.3f A, omega=%.2f rad/s\n",
                   step, 100.0 * step / NUMBER_OF_STEPS,
                   ACM.iDQ[0], ACM.iDQ[1], ACM.varOmega);
        }
    }

    

    clock_t end = clock();
    fclose(fw);

    /* final results */
    printf("\n=== Simulation Complete ===\n");
    printf("Final state at t=%.3f s:\n", ACM.timebase);
    printf("  iD = %.4f A (cmd = %.1f A)\n", ACM.iDQ[0], CMD_ID);
    printf("  iQ = %.4f A (cmd = %.1f A)\n", ACM.iDQ[1], CMD_IQ);
    printf("  omega = %.2f rad/s\n", ACM.varOmega);
    printf("  Tem = %.4f Nm\n", ACM.Tem);

    printf("\nSimulation time: %.3f seconds\n", (REAL)(end - begin) / CLOCKS_PER_SEC);
    printf("Data saved to: %s\n", DATA_FILE_NAME);

    return 0;
}

void init_Machine(void) {
    /* Name plate data */
    ACM.npp = TEST_MOTOR_NPP;
    ACM.npp_inv = 1.0 / ACM.npp;
    ACM.IN = TEST_MOTOR_IN;

    /* Electrical parameters */
    ACM.R = TEST_MOTOR_R;
    ACM.Ld = TEST_MOTOR_LD;
    ACM.Lq = TEST_MOTOR_LQ;
    ACM.KE = TEST_MOTOR_KE;
    ACM.KA = ACM.KE;
    ACM.Rreq = TEST_MOTOR_RREQ;

    /* Mechanical parameters */
    ACM.Js = TEST_MOTOR_JS;
    ACM.Js_inv = 1.0 / ACM.Js;

    /* States initialization */
    ACM.NS = MACHINE_NUMBER_OF_STATES;
    for (int i = 0; i < ACM.NS; i++) {
        ACM.x[i] = 0.0;
        ACM.x_dot[i] = 0.0;
    }

    if (ACM.Rreq <= 0) {
        ACM.x[2] = ACM.KE;
    }

    /* Inputs initialization */
    ACM.uAB_dist[0] = 0.0;
    ACM.uAB_dist[1] = 0.0;
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
    ACM.emf_AB[0] = 0.0;
    ACM.emf_AB[1] = 0.0;
    ACM.iuvw[0] = 0.0;
    ACM.iuvw[1] = 0.0;
    ACM.iuvw[2] = 0.0;
    ACM.Tem = 0.0;
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);
    ACM.cosT_delay_1p5omegaTs = ACM.cosT;
    ACM.sinT_delay_1p5omegaTs = ACM.sinT;
    ACM.powerfactor = 0.0;

    /* Simulation settings */
    ACM.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = MACHINE_SIM_PER_CONTROL;
    ACM.Ts = MACHINE_TS;
    ACM.current_theta = 0.0;
    ACM.voltage_theta = 0.0;

    /* Time */
    ACM.timebase = 0.0;

}

/*
 * Machine Simulation Step
 */
int machine_simulation(void) {
    inverter_model();

    ACM.uDQ[0] = AB2M(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
    ACM.uDQ[1] = AB2T(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);

    RK4(ACM.timebase, ACM.x, ACM.Ts);

    ACM.varOmega = ACM.x[1];
    ACM.varTheta = ACM.x[0];
    ACM.theta_d = ACM.varTheta * ACM.npp;
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);

    ACM.iDQ[0] = ACM.x[3];
    ACM.iDQ[1] = ACM.x[4];

    ACM.iAB[0] = MT2A(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);
    ACM.iAB[1] = MT2B(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);

    ACM.KA = ACM.x[2];
    ACM.psi_AB[0] = ACM.KA * ACM.cosT;
    ACM.psi_AB[1] = ACM.KA * ACM.sinT;

    ACM.emf_AB[0] = ACM.x_dot[2] * ACM.cosT + ACM.KA * (-sin(ACM.theta_d)) * (ACM.npp * ACM.x_dot[0]);
    ACM.emf_AB[1] = ACM.x_dot[2] * ACM.sinT + ACM.KA * cos(ACM.theta_d) * (ACM.npp * ACM.x_dot[0]);

    while (ACM.theta_d > M_PI)  ACM.theta_d -= 2 * M_PI;
    while (ACM.theta_d < -M_PI) ACM.theta_d += 2 * M_PI;

    if (!isNumber(ACM.varOmega)) {
        printf("Warning: varOmega is NaN or Inf, stopping simulation\n");
        return TRUE;
    }

    return FALSE;
}

/*
 * Inverter Model
 */
void inverter_model(void) {
    ACM.uAB_inverter[0] = ACM.uAB[0];
    ACM.uAB_inverter[1] = ACM.uAB[1];
}

/*
 * RK4 Integration
 */
void RK4(REAL t, REAL *x, REAL hs) {
    #define NS MACHINE_NUMBER_OF_STATES

    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];

    DYNAMICS_MACHINE(t, x, fx);
    for (int i = 0; i < NS; i++) {
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i] * 0.5;
    }

    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < NS; i++) {
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i] * 0.5;
    }

    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < NS; i++) {
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }

    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < NS; i++) {
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i]) * one_over_six;
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i]) * one_over_six / hs;
    }

    #undef NS
}

/*
 * Motor Dynamics
 */
void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]) {
    REAL KA = x[2];
    REAL iD = x[3];
    REAL iQ = x[4];

    if (KA == 0.0) {
        ACM.omega_slip = 0.0;
    } else {
        ACM.omega_slip = ACM.Rreq * iQ / KA;
    }

    ACM.omega_syn = x[1] * ACM.npp + ACM.omega_slip;

    if (ACM.Rreq > 0) {
        fx[2] = ACM.Rreq * iD - ACM.Rreq / (ACM.Ld - ACM.Lq) * KA;
        fx[3] = (ACM.uDQ[0] - ACM.R * iD + ACM.omega_syn * ACM.Lq * iQ - fx[2]) / ACM.Lq;
    } else if (ACM.Rreq < 0) {
        fx[2] = 0;
        fx[3] = 0;
    } else {
        fx[2] = 0;
        fx[3] = (ACM.uDQ[0] - ACM.R * iD + ACM.omega_syn * ACM.Lq * iQ) / ACM.Ld;
    }

    fx[4] = (ACM.uDQ[1] - ACM.R * iQ - ACM.omega_syn * ACM.Ld * iD - ACM.omega_syn * KA) / ACM.Lq;

    ACM.Tem = CLARKE_TRANS_TORQUE_GAIN * ACM.npp * KA * iQ;

    fx[0] = x[1] + ACM.omega_slip / ACM.npp;
    fx[1] = (ACM.Tem - ACM.TLoad) / ACM.Js;
}



//-------------------- End of File -------------------------------------------
