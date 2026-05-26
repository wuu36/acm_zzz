// This header file is automatically generated. Any modification to this file will get lost.
#ifndef SUPER_CONFIG_H
#define SUPER_CONFIG_H
#include "typedef.h"

#define WHO_IS_USER 102209 

typedef struct {
    long npp;
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
    REAL CLTS;
    long NUMBER_OF_STEPS;
    long MACHINE_SIM_PER_CONTROL;
    long MODE_SELECT;
    long NUNMBER_OF_STEPS;
    long MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;
    long DOWN_SAMPLE;
} ST_sim;

typedef struct {
    long set_iq_command;
    BOOL verbose;
    long who_is_user;
    long mode_select_synchronous_motor;
    long mode_select_induction_motor;
} ST_user;


typedef struct {
    ST_init init;
    ST_sim sim;
    ST_user user;
} ST_D_SIM;



#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).timebase\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).timebase


#define DATA_FILE_NAME "../dat/SEW100W (SF40B01030C3004).dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // SUPER_CONFIG_H
