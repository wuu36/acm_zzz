//============================================================================
//  Includes
//============================================================================

#include "ACMSim.h"


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

struct ControllerForExperiment CTRL_1;
struct ControllerForExperiment *CTRL;

struct DebugExperiment debug_1;
struct DebugExperiment *debug = &debug_1;


/* 定义顶级结构体(指针的集合) */
ST_D_SIM d_sim;
int axisCnt = 0;


/* 定义内存空间 */
st_motor_parameters t_motor_1={0};
st_enc t_enc_1={0};

st_controller_inputs t_I_1={0};
st_controller_outputs t_O_1={0};


//============================================================================
// Local Function Prototypes
//============================================================================

static void overwrite_di_sim();

//============================================================================
// Function declarations.
//============================================================================

void init_debug(){
    debug = &debug_1;
    (*debug).error = 0;
    (*debug).who_is_user = d_sim.user.who_is_user;
    if(d_sim.init.Rreq>0){
        (*debug).mode_select = d_sim.user.mode_select_induction_motor;
    }else{
        (*debug).mode_select = d_sim.user.mode_select_synchronous_motor;
    }

    (*debug).Overwrite_Current_Frequency = 50.0;
    (*debug).Overwrite_theta_d           = 0.0;

    (*debug).set_id_command = 0.0;
    (*debug).set_iq_command = d_sim.user.set_iq_command;

    (*debug).vvvf_voltage = 3.0;
    (*debug).vvvf_frequency = 5.0;
}

static void overwrite_di_sim(){

}

void init_experiment(){
    overwrite_di_sim();
    init_CTRL();
}

void init_CTRL(){
    allocate_CTRL(CTRL);

    /* basic quantities */
}

void allocate_CTRL(struct ControllerForExperiment *p){
    if(axisCnt==0){
        p->motor = &t_motor_1;
        p->enc = &t_enc_1;

        p->i = &t_I_1;
        p->o = &t_O_1;
    }

}

/* main switch as per MODE_SELECT */
int main_switch(long mode_select){
    static long mode_select_last = 0;
    static int mode_initialized = FALSE;
    if(mode_select != mode_select_last) mode_initialized = FALSE;
    switch (mode_select){
        case MODE_SELECT_PWM_DIRECT: // 1
            // printf("in pwm direct\n");
            (*CTRL).o->cmd_uAB_to_inverter[0] = 10.0;   // Ualpha
            (*CTRL).o->cmd_uAB_to_inverter[1] = 0.0;    // Ubeta
            (*CTRL).svgen1.Ta = 0.7;
            (*CTRL).svgen1.Tb = 0.3;
            (*CTRL).svgen1.Tc = 0.5;
            return 0; 
            break;
        case MODE_SELECT_VOLTAGE_OPEN_LOOP: // 11
            // printf("in voltage open loop\n");
            (*CTRL).o->cmd_uAB_to_inverter[0] = (*debug).vvvf_voltage * cos((*debug).vvvf_frequency*2*M_PI*CTRL->timebase);
            (*CTRL).o->cmd_uAB_to_inverter[1] = (*debug).vvvf_voltage * sin((*debug).vvvf_frequency*2*M_PI*CTRL->timebase);
            
            (*CTRL).i->iDQ[0] = ACM.iDQ[0];
            (*CTRL).i->iDQ[1] = ACM.iDQ[1];
            break;
        case MODE_SELECT_VELOCITY_LOOP: // 4
            printf("in velocity loop\n");
            break;
        case MODE_SELECT_GENERATOR: // 8
            break;
        default:
            printf("in default\n");
            break;
    }



    return 0;
}

/* other only simulation codes */
#if PC_SIMULATION
void _user_time_varying_parameters(){
    // ACM.R  = d_sim.init.R  * 2.5;
    // ACM.Ld = d_sim.init.Ld * 0.25;
    // ACM.Lq = d_sim.init.Lq * 0.25;
    
    // 0. 参数时变
    // if (fabsf((*CTRL).timebase-0.025)<CL_TS){
    //     printf("[Runtime] Rotor inertia of the simulated machine has changed! Js=%g\n", ACM.Js);
        // ACM.Js     = 0.1 * d_sim.init.Js; // kg.m^2 0.41500000000000004
        // ACM.Js_inv = 1.0 / ACM.Js;
    // }
    // if (fabsf((*CTRL).timebase-0.035)<CL_TS){
    //     printf("[Runtime] Rotor inertia of the simulated machine has changed! Js=%g\n", ACM.Js);
    //     ACM.Js     = 0.1 * d_sim.init.Js; // kg.m^2
    //     ACM.Js_inv = 1.0 / ACM.Js;
    // }
    ///
    // Changing KE should go to the init_Machine to change the initial value of KE if u are running at a PMSM Ld = Lq
    ///
}
#endif


//-------------------- End of File -------------------------------------------
