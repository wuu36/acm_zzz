#ifndef _TYPEDEF_H
#define _TYPEDEF_H

/*==============================================================================
    INCLUDES
==============================================================================*/

#include "super_config.h"

/*==============================================================================
    DEFINES
==============================================================================*/

typedef float REAL;


/* boolean type */
typedef int BOOL;
#define TRUE    1
#define FALSE   0

/* mathematical constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* clarke transformation factors */
#define CLARKE_TRANS_TORQUE_GAIN    (1.5)

/* useful constants */
#define one_over_six    (1.0/6.0)

/* unit conversions */

#define MECH_RAD_PER_SEC_2_RPM  (60.0/(2*M_PI))


/*
* Coordinate transformation macros
* 
* Park transformation: alpha-beta to d-q
* AB2M: alpha-beta to d-axis (M for Main/D-axis) 
* AB2T: alpha-beta to q-axis (T for Transverse/Q-axis)
*
* Inverse Park transformation: d-q to alpha-beta
* MT2A: d-q to alpha-axis
* MT2B: d-q to beta-axis
*/
#define AB2M(alpha, beta, cosT, sinT)   ( (alpha)*(cosT) + (beta)*(sinT) )
#define AB2T(alpha, beta, cosT, sinT)   (-(alpha)*(sinT) + (beta)*(cosT) )
#define MT2A(d, q, cosT, sinT)  ( (d)*(cosT) - (q)*(sinT) )
#define MT2B(d, q, cosT, sinT)  ( (d)*(sinT) + (q)*(cosT) )


/* 控制策略 */

#define CL_TS           (d_sim.sim.CLTS)
#define CL_TS_INVERSE   (1.0 / CL_TS)

#define MACHINE_TS          (CL_TS/d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD)
#define MACHINE_TS_INVERSE  (1.0/MACHINE_TS)

/*==============================================================================
    TYPES
==============================================================================*/


/*==============================================================================
    GLOBAL VARIABLE DECLARATIONS
==============================================================================*/


/*==============================================================================
    INLINE FUNCTION DEFINITIONS
==============================================================================*/


/*==============================================================================
    FUNCTION DECLARATIONS
==============================================================================*/



#endif /* _TYPEDEF_H */
/*-------------------- End of File -------------------------------------------*/
