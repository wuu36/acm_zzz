//============================================================================
//  Includes
//============================================================================
#include <stdio.h>
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

struct MachineSimulated ACM;

//============================================================================
// Local Function Prototypes
//============================================================================

//============================================================================
// Function declarations.
//============================================================================

int main(){
    printf("hello world!\n");
    CTRL = &CTRL_1;
    debug = &debug_1;

    init_d_sim();
    init_debug();
    init_experiment();
    init_Machine();

    if(d_sim.user.verbose)print_info();
    FILE *fw; fw = fopen(DATA_FILE_NAME, "w"); write_header_to_file(fw);
    clock_t begin, end; begin = clock(); /* c代码执行时间 */
    int _, dfe_counter=0; /* _ for the outer iteration, dfe_counter for down frequency execution(降频执行变量)*/
    for(_=0; _<d_sim.sim.NUMBER_OF_STEPS*d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD; ++_){
        ACM.timebase += ACM.Ts;
        _user_time_varying_parameters(); /* 用户：定义时变参数 */
        if(machine_simulation()){printf("\t[main.c] break the loop.\n"); break;}
        if(++dfe_counter == d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD){
            dfe_counter = 0; (*CTRL).timebase += CL_TS; // DSP中的时间
            measurement();      // 采样， 包括dsp中的adc采样等
        }
        if(_%100000==0){
            printf("_=%d\n", _);
        }
    }

    end = clock(); if(d_sim.user.verbose)printf("\t[main.c] the simulattion C costs %g sec.\n", (REAL)(end - begin)/CLOCKS_PER_SEC);
    fclose(fw);
    return 0;
}

void measurement(){
    
}

void init_Machine(){
    ACM.npp = d_sim.init.npp;
    ACM.npp_inv = 1.0/ACM.npp;

    // inputs
    
    ACM.uAB_inverter[0] = 0.0;
    ACM.uAB_inverter[1] = 0.0;
    ACM.uAB[0] = 0.0;
    ACM.uAB[1] = 0.0;
    ACM.uDQ[0] = 0.0;
    ACM.uDQ[1] = 0.0;
    
    // outputs

    ACM.omega_syn = 0.0;

    ACM.theta_d = 0.0;

    ACM.cosT_delay_1p5omegaTs = cos(ACM.x[0] * ACM.npp);
    ACM.sinT_delay_1p5omegaTs = sin(ACM.x[0] * ACM.npp);

    ACM.cosT = cos(ACM.x[0]*ACM.npp);
    ACM.sinT = sin(ACM.x[0]*ACM.npp);
    // simulation settings
    ACM.timebase = 0.0;
    ACM.Ts = MACHINE_TS;

}

int machine_simulation(){

    inverter_model();
    if(FALSE){
        // TODO: 这里实际上缺一个延时环节
        ACM.cosT_delay_1p5omegaTs = cosf(ACM.theta_d - 1.5 * ACM.omega_syn *CL_TS);
        ACM.sinT_delay_1p5omegaTs = sinf(ACM.theta_d - 1.5 * ACM.omega_syn *CL_TS);
        ACM.uDQ[0] = AB2M(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT_delay_1p5omegaTs, ACM.sinT_delay_1p5omegaTs);
        ACM.uDQ[1] = AB2T(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT_delay_1p5omegaTs, ACM.sinT_delay_1p5omegaTs);
    }else{
        ACM.uDQ[0] = AB2M(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
        ACM.uDQ[1] = AB2T(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
    }

    // 数值积分

    return FALSE;
}

/* 逆变器建模 */
void inverter_model(){

    // amplitude-invariant to power-invariant
    // 考虑控制器和电机所用Clarke变换不同导致的系数变化

    // 根据给定电压(*CTRL).o->cmd_uAB[0]和实际的电机电流ACM.ial，计算畸变的逆变器输出电压ACM.ual。
    #if __INVERTER_NONLINEARITY == 4
        InverterNonlinearity_ExperimentalLUT_Indexed(uAB_inverter, uAB, ACM.iAB);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
    #elif __INVERTER_NONLINEARITY == 3
        InverterNonlinearity_ExperimentalLUT( (*CTRL).o->cmd_uAB_to_inverter[0], \
                                            (*CTRL).o->cmd_uAB_to_inverter[1], \
                                            ACM.iAB[0], \
                                            ACM.iAB[1]);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - (*CTRL).o->cmd_uAB[0];
        // DIST_BE = ACM.ube - (*CTRL).o->cmd_uAB[1];
    #elif __INVERTER_NONLINEARITY == 2
        InverterNonlinearity_ExperimentalSigmoid( (*CTRL).o->cmd_uAB_to_inverter[0], \
                                            (*CTRL).o->cmd_uAB_to_inverter[1], \
                                            ACM.iAB[0], \
                                            ACM.iAB[1]);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
        printf("ACM.ual_c_dist = %g, ACM.ube_c_dist = %g\n", ACM.ual_c_dist, ACM.ube_c_dist);
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - (*CTRL).o->cmd_uAB[0];
        // DIST_BE = ACM.ube - (*CTRL).o->cmd_uAB[1];
    #elif __INVERTER_NONLINEARITY == 1
        InverterNonlinearity_SKSul96( (*CTRL).o->cmd_uAB_to_inverter[0], \
                                    (*CTRL).o->cmd_uAB_to_inverter[1], \
                                    ACM.ial, \
                                    ACM.ibe);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - (*CTRL).o->cmd_uAB[0];
        // DIST_BE = ACM.ube - (*CTRL).o->cmd_uAB[1];
    #else
        ACM.uAB_inverter[0] = ACM.uAB[0];
        ACM.uAB_inverter[1] = ACM.uAB[1];
    #endif
}


//-------------------- End of File -------------------------------------------
