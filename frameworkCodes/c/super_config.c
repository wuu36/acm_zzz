// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 4;
    d_sim.init.IN = 3.5;
    d_sim.init.R = 0.475;
    d_sim.init.Ld = 0.0020499999999999997;
    d_sim.init.Lq = 0.0020499999999999997;
    d_sim.init.KE = 0.010719670358105074;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 3.5000000000000004e-06;
    d_sim.init.Vdc = 48;

    d_sim.sim.CLTS = 5e-05;
    d_sim.sim.NUMBER_OF_STEPS = 4000;
    d_sim.sim.MACHINE_SIM_PER_CONTROL = 1;
    d_sim.sim.MODE_SELECT = 4;
    d_sim.sim.NUNMBER_OF_STEPS = 4000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.user.set_iq_command = 2;
    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 101616;
    d_sim.user.mode_select_synchronous_motor = 11;
    d_sim.user.mode_select_induction_motor = 32;

}
