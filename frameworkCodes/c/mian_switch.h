#ifndef _MIAN_SWITCH_H
#define _MIAN_SWITCH_H

/*==============================================================================
    INCLUDES
==============================================================================*/

#include "typedef.h"
#include "pi_math.h"

/*==============================================================================
    DEFINES
==============================================================================*/


/*==============================================================================
    TYPES
==============================================================================*/

typedef struct {
    REAL R;         /* Stator resistance Ohm */
    REAL Ld;        /* D-axis inductance H */
    REAL Lq;        /* Q-axis inductance H */
    REAL Ld_inv;    /* 1/Ld */
    REAL Lq_inv;    /* 1/Lq */
    REAL KE;        /* Back-EMF coefficient Wb */
    REAL npp;       /* Number of pole pairs */
    REAL npp_inv;   /* 1/npp */
    REAL Js;        /* Shaft inertia kg.m^2 */
    REAL Js_inv;    /* 1/Js */
} st_motor_parametes;

typedef struct {
    REAL theta_d_elec;  /* electrical angle from encoder */
    REAL varOmega;      /* mechnical speed */
    REAL iAB[2];        /* AB frame currents */
    REAL iDQ[2];        /* DQ frame currents (after Park transform) */
    REAL cmd_varOmega;  /* speed command */
    REAL cmd_iDQ[2];    /* current command in DQ frame */
} st_controller_inputs;

typedef struct {
    REAL cosT;          /* cos(theta_d_elec) */
    REAL sinT;          /* sin(theta_d_elec) */
    REAL omega_syn;     /* synchronous speed */
    
    /* PI controllers */
    PI_CONTROLLER *iD;  /* D-axis current PI */
    PI_CONTROLLER *iQ;  /* Q-axis current PI */
    PI_CONTROLLER *Speed;    /* Speed PI */
    
    /* velcocity loop execution counter */
    int vc_count;
} st_controller_states;

typedef struct {
    REAL cmd_uDQ[2];    /* voltage command in DQ frame */
    REAL cmd_uAB[2];    /* voltage command in AB frame */
} st_controller_outputs;


typedef struct {
    REAL timebase;

    st_motor_parametes *motor;
    st_controller_inputs *i;
    st_controller_states *s;
    st_controller_outputs *o;
} Controller;

/*==============================================================================
    GLOBAL VARIABLE DECLARATIONS
==============================================================================*/
extern Controller CTRL;
extern PI_CONTROLLER PID_iD;
extern PI_CONTROLLER PID_iQ;
extern PI_CONTROLLER PID_Speed;

/*==============================================================================
    INLINE FUNCTION DEFINITIONS
==============================================================================*/


/*==============================================================================
    FUNCTION DECLARATIONS
==============================================================================*/

void init_motor_params(void);
void init_PI_controllers(void);
void init_CTRL(void);


#endif /* _MIAN_SWITCH_H */
/*-------------------- End of File -------------------------------------------*/
