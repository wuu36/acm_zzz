//============================================================================
//  Includes
//============================================================================

#include "ACMSim.h"
#include "main_switch.h"
#include "pi_math.h"
#include <stdio.h>
#include <math.h>
#include <time.h>


//============================================================================
// Local Macros
//============================================================================

/* Simulation parameters */
#define MACHINE_TS (CL_TS / 1.0)
#define NUMBER_OF_STEPS 100000    /* 10秒仿真 */
#define MACHINE_SIM_PER_CONTROL 1

/* Data file path */
#define DATA_FILE_NAME "../dat/test_motor_foc_load.dat"

/* Test conditions */
#define CMD_ID 0.0             /* D-axis current command [A] */
#define CMD_IQ 1.0             /* Q-axis current command [A] */
/* 方案A: 负载匹配电流指令
   TLoad = cmd_iQ × 1.5 × npp × KE = 1.0 × 1.5 × 4 × 0.0107 = 0.0642 Nm */
#define LOAD_TORQUE 0.0642     /* Load torque [Nm] - matches cmd_iQ=1A */
// #define LOAD_TORQUE 0.08     /* Load torque [Nm] - matches cmd_iQ=1A */
#define LOAD_APPLY_TIME 0.1    /* Apply load at t=0.1s */


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

//============================================================================
// Function declarations.
//============================================================================

/*
 * Motor Initialization
 */
void init_Machine(void) {
    ACM.npp = TEST_MOTOR_NPP;
    ACM.npp_inv = 1.0 / ACM.npp;
    ACM.IN = TEST_MOTOR_IN;
    ACM.R = TEST_MOTOR_R;
    ACM.Ld = TEST_MOTOR_LD;
    ACM.Lq = TEST_MOTOR_LQ;
    ACM.KE = TEST_MOTOR_KE;
    ACM.KA = ACM.KE;
    ACM.Rreq = TEST_MOTOR_RREQ;
    ACM.Js = TEST_MOTOR_JS;
    ACM.Js_inv = 1.0 / ACM.Js;

    ACM.NS = MACHINE_NUMBER_OF_STATES;
    for (int i = 0; i < ACM.NS; i++) {
        ACM.x[i] = 0.0;
        ACM.x_dot[i] = 0.0;
    }
    ACM.x[2] = ACM.KE;

    ACM.TLoad = 0.0;
    ACM.theta_d = 0.0;
    ACM.cosT = 1.0;
    ACM.sinT = 0.0;
    ACM.Ts = MACHINE_TS;
    ACM.timebase = 0.0;
}

void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]) {
    REAL KA = x[2];
    REAL iD = x[3];
    REAL iQ = x[4];

    REAL omega_syn = x[1] * ACM.npp;

    fx[2] = 0;
    fx[3] = (ACM.uDQ[0] - ACM.R * iD + omega_syn * ACM.Lq * iQ) / ACM.Ld;
    fx[4] = (ACM.uDQ[1] - ACM.R * iQ - omega_syn * ACM.Ld * iD - omega_syn * KA) / ACM.Lq;

    ACM.Tem = CLARKE_TRANS_TORQUE_GAIN * ACM.npp * KA * iQ;

    fx[0] = x[1];
    fx[1] = (ACM.Tem - ACM.TLoad) / ACM.Js;
}

void RK4(REAL t, REAL *x, REAL hs) {
    REAL k1[5], k2[5], k3[5], k4[5], xk[5], fx[5];

    DYNAMICS_MACHINE(t, x, fx);
    for (int i = 0; i < 5; i++) { k1[i] = fx[i] * hs; xk[i] = x[i] + k1[i]*0.5; }

    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < 5; i++) { k2[i] = fx[i] * hs; xk[i] = x[i] + k2[i]*0.5; }

    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < 5; i++) { k3[i] = fx[i] * hs; xk[i] = x[i] + k3[i]; }

    DYNAMICS_MACHINE(t, xk, fx);
    for (int i = 0; i < 5; i++) {
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i]+k3[i]) + k4[i]) * one_over_six;
    }
}

int machine_simulation(void) {
    ACM.uDQ[0] = AB2M(ACM.uAB[0], ACM.uAB[1], ACM.cosT, ACM.sinT);
    ACM.uDQ[1] = AB2T(ACM.uAB[0], ACM.uAB[1], ACM.cosT, ACM.sinT);

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

    while (ACM.theta_d > M_PI) ACM.theta_d -= 2*M_PI;
    while (ACM.theta_d < -M_PI) ACM.theta_d += 2*M_PI;

    return !isNumber(ACM.varOmega);
}


int main(void) {
    printf("=== FOC Test with Load ===\n");
    printf("验证: cmd_iQ=%.1f A, TLoad=%.3f Nm (t=%.2f s)\n\n", CMD_IQ, LOAD_TORQUE, LOAD_APPLY_TIME);
    
    init_Machine();
    init_CTRL();

    printf("PI: Kp=%.2f, Ki=%.4f, Umax=%.1f V\n\n", PID_iQ.Kp, PID_iQ.Ki, PID_iQ.Umax);

    FILE *fw = fopen(DATA_FILE_NAME, "w");
    fprintf(fw, "time,omega,iD,iQ,uD,uQ,cmd_iD,cmd_iQ,Tem,TLoad\n");

    clock_t begin = clock();

    int load_applied = 0;

    for (int step = 0; step < NUMBER_OF_STEPS; step++) {
        ACM.timebase = step * CL_TS;

        if (ACM.timebase >= LOAD_APPLY_TIME && !load_applied) {
            ACM.TLoad = LOAD_TORQUE;
            load_applied = 1;
            printf(">>> Load applied: TLoad=%.3f Nm at t=%.3f s\n", ACM.TLoad, ACM.timebase);
        }

        CTRL_inputs.theta_d_elec = ACM.theta_d;
        CTRL_inputs.iAB[0] = ACM.iAB[0];
        CTRL_inputs.iAB[1] = ACM.iAB[1];
        CTRL_inputs.cmd_iDQ[0] = CMD_ID;
        CTRL_inputs.cmd_iDQ[1] = CMD_IQ;

        /* FOC控制 */
        main_switch(MODE_SELECT_FOC);

        ACM.uAB[0] = CTRL_outputs.cmd_uAB[0];
        ACM.uAB[1] = CTRL_outputs.cmd_uAB[1];

        if (machine_simulation()) break;

        fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
            ACM.timebase, ACM.varOmega, ACM.iDQ[0], ACM.iDQ[1],
            CTRL_outputs.cmd_uDQ[0], CTRL_outputs.cmd_uDQ[1],
            CMD_ID, CMD_IQ, ACM.Tem, ACM.TLoad);

        if (step % 20000 == 0) {
            printf("t=%.2f s: omega=%.1f rad/s, iQ=%.3f A, Tem=%.4f Nm\n",
                   ACM.timebase, ACM.varOmega, ACM.iDQ[1], ACM.Tem);
        }
    }

    clock_t end = clock();
    fclose(fw);
    
    printf("\n=== Results ===\n");
    printf("Final: omega=%.2f rad/s, iQ=%.4f A (cmd=%.1f), Tem=%.4f Nm\n",
           ACM.varOmega, ACM.iDQ[1], CMD_IQ, ACM.Tem);
    printf("iQ error: %.2f%%\n", fabs(ACM.iDQ[1]-CMD_IQ)/CMD_IQ*100);
    printf("Time: %.3f s\n\n", (REAL)(end-begin)/CLOCKS_PER_SEC);
}

//-------------------- End of File -------------------------------------------
