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
            write_data_to_file(fw);

            #if WHO_IS_USER == USER_BEZIER
            #endif
            main_switch((*debug).mode_select);
            if((*debug).mode_select != MODE_SELECT_GENERATOR){
                ACM.uAB[0] = (*CTRL).o->cmd_uAB_to_inverter[0];
                ACM.uAB[1] = (*CTRL).o->cmd_uAB_to_inverter[1];
            }
        }
        if(_==0 || _==1 || _==5 || _==50 || _==1000){
            printf("_=%d uAB=[%.2f,%.2f] uABinv=[%.2f,%.2f] uDQ=[%.2f,%.2f] x[1]=%.6f x[2]=%.6f x[3]=%.6f x[4]=%.6f Tem=%.6f\n", 
                _, ACM.uAB[0], ACM.uAB[1], ACM.uAB_inverter[0], ACM.uAB_inverter[1],
                ACM.uDQ[0], ACM.uDQ[1], ACM.x[1], ACM.x[2], ACM.x[3], ACM.x[4], ACM.Tem);
        }
    }

    end = clock(); if(d_sim.user.verbose)printf("\t[main.c] the simulattion C costs %g sec.\n", (REAL)(end - begin)/CLOCKS_PER_SEC);
    fclose(fw);
    return 0;
}

void measurement(){
    // 本函数每隔采样时间 CL_TS 执行一次

    // 下面出现的US_C, IS_C等，都是全局的宏变量，方便在不同的.c文件内共享

    // 电压测量
    (*CTRL).i->Vdc = d_sim.init.Vdc;
    #define CURRENT_OFFSET_A 0//0.1//0.2//0.05
    #define CURRENT_OFFSET_B 0//0.1//0.2//0.05
    (*CTRL).i->iAB[0] = ACM.iAB[0] + 1*CURRENT_OFFSET_A;
    (*CTRL).i->iAB[1] = ACM.iAB[1] + 1*CURRENT_OFFSET_B;

    // exact measurement of d-axis angle
    (*CTRL).i->varTheta = ACM.varTheta;
    (*CTRL).i->varOmega = ACM.varOmega;
    
}

void init_Machine(){
    ACM.npp = d_sim.init.npp;
    ACM.npp_inv = 1.0/ACM.npp;

    // electrical parameters
    ACM.R = d_sim.init.R * 1.0;
    ACM.Ld = d_sim.init.Ld;
    ACM.Lq = d_sim.init.Lq * 1.0;
    ACM.KE = d_sim.init.KE * 1.0;
    ACM.KA = ACM.KE;
    ACM.Rreq = d_sim.init.Rreq;

    // mechanical parameters
    ACM.Js = d_sim.init.Js; // kg*m^2
    ACM.Js_inv = 1.0 / ACM.Js;

    // states
    ACM.NS = MACHINE_NUMBER_OF_STATES;
    int i;
    for(i=0;i<ACM.NS;++i){
        ACM.x[i] = 0.0;
        ACM.x_dot[i] = 0.0;
    }
    if(ACM.Rreq <= 0){
        ACM.x[2] = ACM.KE; /* PMSM: KA = KE */
    }

    // inputs
    
    ACM.uAB_inverter[0] = 0.0;
    ACM.uAB_inverter[1] = 0.0;
    ACM.uAB[0] = 0.0;
    ACM.uAB[1] = 0.0;
    ACM.uDQ[0] = 0.0;
    ACM.uDQ[1] = 0.0;
    ACM.TLoad = 0;
    
    // outputs
    ACM.varTheta = 0.0;
    ACM.varOmega = 0.0;

    ACM.omega_syn = 0.0;
    ACM.omega_slip = 0.0;
    ACM.theta_d = 0.0;
    ACM.iDQ[0] = 0.0;
    ACM.iDQ[1] = 0.0;
    ACM.iAB[0] = 0.0;
    ACM.iAB[1] = 0.0;

    ACM.Tem = 0.0;
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
    RK4(ACM.timebase, ACM.x, ACM.Ts);
    
    // 电机转速接口
    ACM.varOmega = ACM.x[1];

    // 电机转子位置接口
    // get M-T frame quantities for fun
    ACM.varTheta = ACM.x[0];
    ACM.theta_d = ACM.varTheta*ACM.npp;
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);

    // 电机电流接口
    ACM.iDQ[0] = ACM.x[3];
    ACM.iDQ[1] = ACM.x[4];
    ACM.iAB[0] = MT2A(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);
    ACM.iAB[1] = MT2B(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);
    
    // 电机磁链接口
    ACM.KA = ACM.x[2];
    ACM.psi_AB[0] = ACM.KA * ACM.cosT;
    ACM.psi_AB[1] = ACM.KA * ACM.sinT;
    ACM.emf_AB[0] = ACM.x_dot[2] * ACM.cosT + ACM.KA * -sin(ACM.theta_d) * (ACM.npp * ACM.x_dot[0]);
    ACM.emf_AB[1] = ACM.x_dot[2] * ACM.sinT + ACM.KA *  cos(ACM.theta_d) * (ACM.npp * ACM.x_dot[0]);
    
    // 转子(假想)d轴位置限幅
    while(ACM.theta_d > M_PI) ACM.theta_d -= 2*M_PI;
    while(ACM.theta_d < -M_PI) ACM.theta_d += 2*M_PI;  // 反转
    
    // 简单的程序跑飞检测，比如电机转速无穷大则停止程序
    if(isNumber(ACM.varOmega)){
        return FALSE;
    }else{
        printf("ACM.varOmega is %g\n", ACM.varOmega);
        return TRUE;
    }


    return FALSE;
}

void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]){

    // varTheta = x[0]
    // varOmega = x[1]
    // ACM.theta_d_elec = x[0]*ACM.npp
    // ACM.omega_r = x[1]*ACM.npp
    REAL KA = x[2];
    REAL iD = x[3];
    REAL iQ = x[4];
    if(KA == 0.0){
        ACM.omega_slip = 0.0;
    }else{
        ACM.omega_slip = ACM.Rreq * iQ / KA;
    }
    ACM.omega_syn  = x[1]*ACM.npp + ACM.omega_slip;

    // 电磁子系统 (KA, iD, iQ as x[2], x[3], x[4])
    if (ACM.Rreq > 0){
        // s KA
        fx[2] = ACM.Rreq*iD - ACM.Rreq / (ACM.Ld - ACM.Lq) * KA; // [Apply Park Transorm to (31b)]
        // s iD
        fx[3] = (ACM.uDQ[0] - ACM.R*iD + ACM.omega_syn*ACM.Lq*iQ - fx[2]) / ACM.Lq; // (6a)
    }else if (ACM.Rreq < 0){
        printf("ACM.Rreq is used to calculate slip so it must be zero for PMSM.");
    }else{
            // note fx[3] * ACM.Lq = ACM.uDQ[0] - ACM.R*iD + omega*ACM.Lq*iQ - fx[2]
            //  =>  fx[3] * ACM.Lq = ACM.uDQ[0] - ACM.R*iD + omega*ACM.Lq*iQ - (ACM.Ld - ACM.Lq) * fx[3] - 0.0
            //  =>  fx[3] * ACM.Ld = ACM.uDQ[0] - ACM.R*iD + omega*ACM.Lq*iQ
            //  =>  s iD
        // s iD
        fx[3] = (ACM.uDQ[0] - ACM.R*iD + ACM.omega_syn*ACM.Lq*iQ) / ACM.Ld;
        // s KA 
        fx[2] = (ACM.Ld - ACM.Lq) * fx[3] + 0.0;
    }
    // s iQ
    fx[4] = (ACM.uDQ[1] - ACM.R*iQ - ACM.omega_syn*ACM.Lq*iD - ACM.omega_syn * KA) / ACM.Lq;
    // printf("%g, %g, %g, %g,  |  %g, %g, %g, %g \n",
    //     ACM.timebase, ACM.uDQ[1], iQ, ACM.omega_syn, iD, ACM.omega_syn, KA, fx[4]
    // );

    // 机械子系统 (varTheta, varOmega as x[0], x[1])
    ACM.Tem = CLARKE_TRANS_TORQUE_GAIN * ACM.npp * KA * iQ; // 电磁转矩计算
    fx[0] = x[1] + ACM.omega_slip / ACM.npp; // mech. angular rotor position (accumulated)
    fx[1] = (ACM.Tem - ACM.TLoad) / ACM.Js;  // mech. angular rotor speed
}


void RK4(REAL t, REAL *x, REAL hs){ // 四阶龙格库塔法
    #define NS MACHINE_NUMBER_OF_STATES

    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;

    DYNAMICS_MACHINE(t, x, fx); // timer.t,
    for(i=0;i<NS;++i){        
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }
    
    DYNAMICS_MACHINE(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }
    
    DYNAMICS_MACHINE(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }
    
    DYNAMICS_MACHINE(t, xk, fx); // timer.t+hs, 
    for(i=0;i<NS;++i){        
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;
        // derivatives
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six / hs; 
    }
    #undef NS
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
