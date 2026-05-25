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
#define USER_TEST   101616

#define MA_SEQUENCE_LENGTH  2   // note MA_SEQUENCE_LENGTH * CL_TS = window of moving average in seconds

/*==============================================================================
    TYPES
==============================================================================*/



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
struct ControllerForExperiment{

    /* basic quantities */
    REAL timebase;

    /* machine parameters */
    st_motor_parameters *motor;

    /* peripheral configurations */
    st_enc *enc;
};


struct DebugExperiment{
    long error;
    long who_is_user;
    long mode_select;
    REAL Overwrite_Current_Frequency;
    REAL Overwrite_theta_d;

    

    REAL set_id_command;
    REAL set_iq_command;
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

#endif /* _MAIN_SWITCH_H */
/*-------------------- End of File -------------------------------------------*/
