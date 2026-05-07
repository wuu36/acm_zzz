//============================================================================
//  Includes
//============================================================================

#include <stdio.h>
#include "ACMSim.h"
#include "mian_switch.h"
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

//-------------------- End of File -------------------------------------------
