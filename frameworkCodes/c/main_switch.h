#ifndef _MAIN_SWITCH_H
#define _MAIN_SWITCH_H

/*==============================================================================
    INCLUDES
==============================================================================*/
#include "typedef.h"

/*==============================================================================
    DEFINES
==============================================================================*/
/* define in st_interact_zzz.py */
#define USER_CJH    101976
#define USER_XM     102209
#define USER_BEZIER 224
#define USER_TEST   101616

#define MODE_SELECT_PWM_DIRECT          1
#define MODE_SELECT_VOLTAGE_OPEN_LOOP   11
#define MODE_SELECT_VELOCITY_LOOP       4
#define MODE_SELECT_GENERATOR           8

#define MA_SEQUENCE_LENGTH              2   // note MA_SEQUENCE_LENGTH * CL_TS = window of moving average in seconds

/*==============================================================================
    TYPES
==============================================================================*/

typedef struct {
    REAL  Ualpha; // Input: reference alpha-axis phase voltage
    REAL  Ubeta;  // Input: reference beta-axis phase voltage
    REAL  Ta;     // Output: reference phase-a switching function
    REAL  Tb;     // Output: reference phase-b switching function
    REAL  Tc;     // Output: reference phase-c switching function
    REAL CMPA[3]; // PWM compare register value original
    REAL CMPA_DBC[3]; // PWM compare register value dead-band compensated
    REAL utilization_ratio; //* The squart of the sum of the square of Ualpha and Ubeta
} SVGENDQ;

typedef struct {
    // electrical
    REAL R;
    REAL K;
    REAL Ld;
    REAL Ld_inv;
} st_motor_parameters;

typedef struct {
    /* moving average for speed calculation */
    REAL MA_qepPosCnt[MA_SEQUENCE_LENGTH];
    REAL sum_qepPosCnt;
} st_enc; // encoder

typedef struct {
    // voltage commands
    REAL cmd_uAB[4];
    REAL cmd_uDQ[4];
    REAL cmd_uAB_to_inverter[4];    // foc control output + inverter nolinearity compesation
    REAL cmd_uDQ_to_inverter[4];
    // current commands
    REAL cmd_iAB[4];


} st_controller_outputs;
struct ControllerForExperiment{

    /* basic quantities */
    REAL timebase;

    /* machine parameters */
    st_motor_parameters *motor;

    /* peripheral configurations */
    st_enc *enc;

    /* inverter */
    
    SVGENDQ svgen1;
    SVGENDQ svgen2;


    /* black box model | controller quantities */

    st_controller_outputs *o;
};


struct DebugExperiment{
    long error;
    long who_is_user;
    long mode_select;
    REAL Overwrite_Current_Frequency;
    REAL Overwrite_theta_d;

    

    REAL set_id_command;
    REAL set_iq_command;

    REAL vvvf_voltage;
    REAL vvvf_frequency;
};

/*==============================================================================
    GLOBAL VARIABLE DECLARATIONS
==============================================================================*/

extern struct ControllerForExperiment CTRL_1;

extern struct ControllerForExperiment *CTRL;

extern struct DebugExperiment debug_1;
extern struct DebugExperiment *debug;

/*==============================================================================
    INLINE FUNCTION DEFINITIONS
==============================================================================*/


/*==============================================================================
    FUNCTION DECLARATIONS
==============================================================================*/

/* user specified functions */
void init_debug();
void init_experiment();
void init_CTRL();
void _user_time_varying_parameters();

void allocate_CTRL(struct ControllerForExperiment *p);


int main_switch(long mode_select);

#endif /* _MAIN_SWITCH_H */
/*-------------------- End of File -------------------------------------------*/
