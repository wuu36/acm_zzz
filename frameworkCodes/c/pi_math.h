#ifndef _PI_MATH_H
#define _PI_MATH_H

/*==============================================================================
    INCLUDES
==============================================================================*/

#include "typedef.h"

/*==============================================================================
    DEFINES
==============================================================================*/


/*==============================================================================
    TYPES
==============================================================================*/

/* PI Controller Structure (TI Style) */
typedef struct {
    REAL  Ref;      // Input: reference set-point
    REAL  Fbk;      // Input: feedback
    REAL  Out;      // Output: controller output
    REAL  Kp;       // Parameter: proportional loop gain
    REAL  Ki;       // Parameter: integral gain (Ki = Kp * Ts / Ti)
    REAL  Umax;     // Parameter: upper saturation limit
    REAL  Umin;     // Parameter: lower saturation limit
    REAL  up;       // Data: proportional term
    REAL  ui;       // Data: integral term
    REAL  v1;       // Data: pre-saturated controller output
    REAL  i1;       // Data: integrator storage: ui(k-1)
} PI_CONTROLLER;

/* default initialization values */
#define PI_CONTROLLER_DEFAULTS { \
    0,      /* Ref */ \
    0,      /* Fbk */ \
    0,      /* Out */ \
    1.0,    /* Kp */ \
    0.0,    /* Ki */ \
    1.0,    /* Umax */ \
    -1.0,   /* Umin */ \
    0.0,    /* up */ \
    0.0,    /* ui */ \
    0.0,    /* v1 */ \
    0.0     /* i1 */ \
}

/* Saturation macro */
#define _IQsat(A, Pos, Neg)  (fmaxf(((fminf((A),(Pos)))),(Neg)))

/* TI Style PI Macro - Grando Anti-Windup */
#define PI_MACRO(v) \
    /* proportional term */ \
    v.up = (v.Kp * (v.Ref - v.Fbk)); \
    \
    /* integral term with anti-windup */ \
    v.ui = (v.Out == v.v1) ? (v.Ki * v.up + v.i1) : v.i1; \
    v.i1 = v.ui; \
    \
    /* control output */ \
    v.v1 = v.up + v.ui; \
    v.Out = _IQsat(v.v1, v.Umax, v.Umin);


/*==============================================================================
    GLOBAL VARIABLE DECLARATIONS
==============================================================================*/


/*==============================================================================
    INLINE FUNCTION DEFINITIONS
==============================================================================*/


/*==============================================================================
    FUNCTION DECLARATIONS
==============================================================================*/



#endif /* _PI_MATH_H */
/*-------------------- End of File -------------------------------------------*/
