// This header file is automatically generated. Any modification to this file will get lost.
#ifndef SUPER_CONFIG_H
#define SUPER_CONFIG_H
#include "typedef.h"

#define WHO_IS_USER 101616 

typedef struct {
    REAL npp;
    REAL IN;
    REAL R;
    REAL Ld;
    REAL Lq;
    REAL KE;
    REAL Rreq;
    REAL Js;
    long Vdc;
} ST_init;

typedef struct {
    REAL CL_TS;
    REAL NUMBER_OF_STEPS;
    REAL MACHINE_SIM_PER_CONTROL;
    REAL MODE_SELECT;
    long NUNMBER_OF_STEPS;
} ST_sim;

typedef struct {
    long who_is_user;
    long mode_select_synchronous_motor;
    long mode_select_induction_motor;
} ST_user;


typedef struct {
    ST_init init;
    ST_sim sim;
    ST_user user;
} ST_D_SIM;



#define DATA_FORMAT "%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1]\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1]


#define DATA_FILE_NAME "../dat/SEW100W (SF40B01030C3004).dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // SUPER_CONFIG_H
