//============================================================================
//  Includes
//============================================================================

#include "ACMSim.h"
#include "main_switch.h"
#include "super_config.h"
#include "math.h"


//============================================================================
// Local Macros
//============================================================================

#define VL_EXE_PER_CL_EXE 10    /* speed loop runs 10x slower than current loop */

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
    motor_params.R = d_sim.motor.R;
    motor_params.Ld = d_sim.motor.Ld;
    motor_params.Lq = d_sim.motor.Lq;
    motor_params.Ld_inv = 1.0 / d_sim.motor.Ld;
    motor_params.Lq_inv = 1.0 / d_sim.motor.Lq;
    motor_params.KE = d_sim.motor.KE;
    motor_params.npp = d_sim.motor.npp;
    motor_params.npp_inv = 1.0 / d_sim.motor.npp;
    motor_params.Js = d_sim.motor.Js;
    motor_params.Js_inv = 1.0 / d_sim.motor.Js;

    CTRL.motor = &motor_params;
}

void init_PI_controllers(void) {
    REAL BW_current = 100.0;    /* current loop bandwidth Hz (保守设置)*/
    REAL BW_speed = 10.0;       /* Speed loop bandwidth Hz (保守设置) */

    /* D-axis current PI */
    PID_iD.Kp = motor_params.Ld * BW_current * 2 * M_PI;
    PID_iD.Ki = PID_iD.Kp * motor_params.R / motor_params.Ld * CL_TS;
    PID_iD.Umax = d_sim.motor.Vdc / 2.0;
    PID_iD.Umin = -PID_iD.Umax;

    /* Q-axis current PI */
    PID_iQ.Kp = motor_params.Lq * BW_current * 2 * M_PI;
    PID_iQ.Ki = PID_iQ.Kp * motor_params.R / motor_params.Lq * CL_TS;
    PID_iQ.Umax = d_sim.motor.Vdc / 2.0;
    PID_iQ.Umin = -PID_iQ.Umax;

    /* Speed PI */
    PID_Speed.Kp = motor_params.Js * BW_speed * 2 * M_PI;
    PID_Speed.Ki = PID_Speed.Kp * BW_speed * 2 * M_PI * CL_TS;
    PID_Speed.Umax = d_sim.motor.IN;  /* Limit to rated current */
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
    CTRL_inputs.cmd_uDQ[0] = 0.0;
    CTRL_inputs.cmd_uDQ[1] = 0.0;

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

void main_switch(long mode_select) {
    switch (mode_select)
    {
    case MODE_SELECT_PWM_DIRECT:
        /* direct PWM output - bypass all control */
        break;
    case MODE_SELECT_VOLTAGE_OPEN_LOOP:
        CTRL_states.cosT = cos(CTRL_inputs.theta_d_elec);
        CTRL_states.sinT = sin(CTRL_inputs.theta_d_elec);
        CTRL_outputs.cmd_uDQ[0] = CTRL_inputs.cmd_uDQ[0];
        CTRL_outputs.cmd_uDQ[1] = CTRL_inputs.cmd_uDQ[1];
        CTRL_outputs.cmd_uAB[0] = MT2A(CTRL_outputs.cmd_uDQ[0], CTRL_outputs.cmd_uDQ[1],
                                       CTRL_states.cosT, CTRL_states.sinT);
        CTRL_outputs.cmd_uAB[1] = MT2B(CTRL_outputs.cmd_uDQ[0], CTRL_outputs.cmd_uDQ[1],
                                       CTRL_states.cosT, CTRL_states.sinT);
        break;
    case MODE_SELECT_FOC:
        /* FOC current loop only */
        _onlyFOC(CTRL_inputs.theta_d_elec, CTRL_inputs.iAB);
        break;
    case MODE_SELECT_VELOCITY_LOOP:
        /* velocity loop + FOC */
        FOC_with_velocity_control(CTRL_inputs.theta_d_elec,
                                CTRL_inputs.varOmega,
                                CTRL_inputs.cmd_varOmega,
                                CTRL_inputs.iAB);
        break;
    case MODE_SELECT_POSITION_LOOP:
        printf("position loop not implemented in phase 2");
    default:
        printf("unknown loop not implemented in phase 2\n");
        break;
    }
}

void _onlyFOC(REAL theta_d_elec, REAL iAB[2]) {
    /* step 1: park transform - convert AB currents to DQ */
    CTRL_states.cosT = cos(theta_d_elec);
    CTRL_states.sinT = sin(theta_d_elec);

    CTRL_inputs.iDQ[0] = AB2M(iAB[0], iAB[1], CTRL_states.cosT, CTRL_states.sinT);
    CTRL_inputs.iDQ[1] = AB2T(iAB[0], iAB[1], CTRL_states.cosT, CTRL_states.sinT);

    /* step 2: Current PI controllers */
    PID_iD.Ref = CTRL_inputs.cmd_iDQ[0];
    PID_iD.Fbk = CTRL_inputs.iDQ[0];
    PI_MACRO(PID_iD);

    PID_iQ.Ref = CTRL_inputs.cmd_iDQ[1];
    PID_iQ.Fbk = CTRL_inputs.iDQ[1];
    PI_MACRO(PID_iQ);

    /* step 3: Output voltage (no decoupling for simplicity) */
    CTRL_outputs.cmd_uDQ[0] = PID_iD.Out;
    CTRL_outputs.cmd_uDQ[1] = PID_iQ.Out;

    /* step 4: Inverse Park transform - convert DQ voltages to AB */
    CTRL_outputs.cmd_uAB[0] = MT2A(CTRL_outputs.cmd_uDQ[0], CTRL_outputs.cmd_uDQ[1],
                                   CTRL_states.cosT, CTRL_states.sinT);
    CTRL_outputs.cmd_uAB[1] = MT2B(CTRL_outputs.cmd_uDQ[0], CTRL_outputs.cmd_uDQ[1],
                                   CTRL_states.cosT, CTRL_states.sinT);
}

void FOC_with_velocity_control(REAL theta_d_elec, REAL varOmega,
                                REAL cmd_varOmega, REAL iAB[2]) {
    /* step1: velocity controller generates iq command */
    CTRL_inputs.cmd_iDQ[1] = _velocityController(cmd_varOmega, varOmega);

    /* step2: id command (default: 0 for surface PMSM) */
    CTRL_inputs.cmd_iDQ[0] = 0.0;

    /* step 3: FOC current loop */
    _onlyFOC(theta_d_elec, iAB);
}

REAL _velocityController(REAL cmd_varOmega, REAL varOmega) {
    /* execute speed PI at lower rate */
    if (++CTRL_states.vc_count >= VL_EXE_PER_CL_EXE) {
        CTRL_states.vc_count = 0;

        PID_Speed.Ref = cmd_varOmega;
        PID_Speed.Fbk = varOmega;
        PI_MACRO(PID_Speed);
    }

    return PID_Speed.Out;
}

//-------------------- End of File -------------------------------------------
