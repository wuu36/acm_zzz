//============================================================================
//  Includes
//============================================================================

#include "ACMSim.h"
#include "mian_switch.h"


//============================================================================
// Local Macros
//============================================================================

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

Controller CTRL;

PI_CONTROLLER PID_iD = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER PID_iQ = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER PID_Speed = PI_CONTROLLER_DEFAULTS;

st_motor_parametes motor_params;

st_controller_inputs CTRL_inputs;
st_controller_states CTRL_states;
st_controller_outputs CTRL_outputs;

//============================================================================
// Local Function Prototypes
//============================================================================

//============================================================================
// Function declarations.
//============================================================================

void init_motor_params(void) {
    motor_params.R = TEST_MOTOR_R;
    motor_params.Ld = TEST_MOTOR_LD;
    motor_params.Lq = TEST_MOTOR_LQ;
    motor_params.Ld_inv = 1.0 / TEST_MOTOR_LD;
    motor_params.Lq_inv = 1.0 / TEST_MOTOR_LQ;
    motor_params.KE = TEST_MOTOR_KE;
    motor_params.npp = TEST_MOTOR_NPP;
    motor_params.npp_inv = 1.0 / TEST_MOTOR_NPP;
    motor_params.Js = TEST_MOTOR_JS;
    motor_params.Js_inv = 1.0 / TEST_MOTOR_JS;

    CTRL.motor = &motor_params;
}

void init_PI_controllers(void) {
    REAL BW_current = 100.0;    /* current loop bandwidth Hz (保守设置)*/
    REAL BW_speed = 10.0;       /* Speed loop bandwidth Hz (保守设置) */

    /* D-axis current PI */
    PID_iD.Kp = motor_params.Ld * BW_current * 2 * M_PI;
    PID_iD.Ki = PID_iD.Kp * motor_params.R / motor_params.Ld * CL_TS;
    PID_iD.Umax = TEST_MOTOR_VDC / 2.0;
    PID_iD.Umin = -PID_iD.Umax;

    /* Q-axis current PI */
    PID_iQ.Kp = motor_params.Lq * BW_current * 2 * M_PI;
    PID_iQ.Ki = PID_iQ.Kp * motor_params.R / motor_params.Lq * CL_TS;
    PID_iQ.Umax = TEST_MOTOR_VDC / 2.0;
    PID_iQ.Umin = -PID_iQ.Umax;

    /* Speed PI */
    PID_Speed.Kp = motor_params.Js * BW_speed * 2 * M_PI;
    PID_Speed.Ki = PID_Speed.Kp * BW_speed * 2 * M_PI * CL_TS;
    PID_Speed.Umax = TEST_MOTOR_IN;  /* Limit to rated current */
    PID_Speed.Umin = -PID_Speed.Umax;

    /* assign PI controllers to state structure */
    CTRL_states.iD = &PID_iD;
    CTRL_states.iQ = &PID_iQ;
    CTRL_states.Speed = &PID_Speed;

    CTRL_states.vc_count = 0;
}


void init_CTRL(void) {
    init_motor_params();
    init_PI_controllers();

    CTRL.i = &CTRL_inputs;
    CTRL.s = &CTRL_states;
    CTRL.o = &CTRL_outputs;

    CTRL.timebase = 0.0;

    /* Initialize inputs */
    CTRL_inputs.theta_d_elec = 0.0;
    CTRL_inputs.varOmega = 0.0;
    CTRL_inputs.iAB[0] = 0.0;
    CTRL_inputs.iAB[1] = 0.0;
    CTRL_inputs.iDQ[0] = 0.0;
    CTRL_inputs.iDQ[1] = 0.0;
    CTRL_inputs.cmd_varOmega = 0.0;
    CTRL_inputs.cmd_iDQ[0] = 0.0;
    CTRL_inputs.cmd_iDQ[1] = 0.0;

    /* Initialize states */
    CTRL_states.cosT = 1.0;
    CTRL_states.sinT = 0.0;
    CTRL_states.omega_syn = 0.0;

    /* Initialize outputs */
    CTRL_outputs.cmd_uDQ[0] = 0.0;
    CTRL_outputs.cmd_uDQ[1] = 0.0;
    CTRL_outputs.cmd_uAB[0] = 0.0;
    CTRL_outputs.cmd_uAB[1] = 0.0;
}

//-------------------- End of File -------------------------------------------
