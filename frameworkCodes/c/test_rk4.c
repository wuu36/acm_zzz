/*
* RK4 numberical intergration accuracy test
* test equation: dx/dt = x, x(0) = 1
* analytical solution: x(t) = e^t
* at t = 1: x(1) = e = 2.71828...
*/

#include <stdio.h>
#include <math.h>

typedef double REAL;

/* Test dynamics: dx/dt = x */
void test_dynamics(REAL t, REAL x, REAL *dx) {
    *dx = x;
}

/* RK4 integration */
void rk4_step(REAL t, REAL *x, REAL h) {
    REAL k1, k2, k3, k4, xk;

    test_dynamics(t, *x, &k1);
    k1 *= h;
    xk = *x + k1 * 0.5;

    test_dynamics(t + h*0.5, xk, &k2);
    k2 *= h;
    xk = *x + k2 * 0.5;

    test_dynamics(t + h*0.5, xk, &k3);
    k3 *= h;
    xk = *x + k3;

    test_dynamics(t + h, xk, &k4);
    k4 *= h;

    *x = *x + (k1 + 2*k2 + 2*k3 + k4) / 6.0;
}


int main(void) {
    printf("=== RK4 Numerical Intergration Test ===\n\n");
    
    REAL x = 1.0;
    REAL h = 0.01;
    REAL t = 0.0;
    int steps = 100;

    printf("Test equation: dx/dt = x, x(0) = 1\n");
    printf("Analytical solution: x(t) = e^t\n");
    printf("Step size h = %g, steps = %d\n\n", h, steps);

    for (int i = 0; i < steps; i++) {
        rk4_step(t, &x, h);
        t += h;

        if (i % 10 == 9) {
            REAL analytical = exp(t);
            REAL error = fabs(x - analytical);
            printf("t = %.2f: RK4 x = %.6, Analytical = %.6f, Error = %.2e\n",
                t, x, analytical, error);
        }
    }

    /* Final result */
    REAL analytical_final = exp(1.0);
    REAL final_error = fabs(x - analytical_final);
    REAL relative_error = final_error / analytical_final * 100;

    printf("\n=== Final Result ===\n");
    printf("At t = 1.0:\n");
    printf("  RK4 result:    x = %.8f\n", x);
    printf("  Analytical:    x = %.8f (e)\n", analytical_final);
    printf("  Absolute error: %.2e\n", final_error);
    printf("  Relative error: %.4f %%\n", relative_error);

    /* Acceptance criteria */
    if (relative_error < 0.01) {
        printf("\n[PASS] RK4 accuracy verified (error < 0.01%%)\n");
        return 0;
    } else {
        printf("\n[FAIL] RK4 accuracy insufficient (error > 0.01%%)\n");
        return 1;
    }
}