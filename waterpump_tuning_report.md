# Automotive Water Pump FOC Parameter Tuning Report

## Motor Specifications

| Parameter | Value |
|-----------|-------|
| Application | Automotive electronic water pump |
| Power | 100-150W |
| DC Bus Voltage | 13.5 V (12V system) |
| Max Phase Voltage | 7.79 V (linear modulation) |
| Pole pairs | 4 |
| Phase Resistance | 0.080 Ohm |
| Ld | 0.15 mH |
| Lq | 0.18 mH |
| PM Flux (KE) | 0.0045 Wb |
| Inertia | 0.00008 kg.m^2 |
| Target Speed | 3500.0 rpm |

## Experiment Results Summary

| Exp | Description | Steady Error | Overshoot | Rise Time | Speed Std | max_iq | max_uq |
|-----|-------------|--------------|----------|-----------|-----------|--------|--------|
| wp_exp0_baseline | Baseline conservative params | 42.39% | 0.0% | 0.000s | 2.2rpm | 8.7A | 7.5V |
| wp_exp1_raise_voltage | Increase voltage limit | 41.31% | 0.0% | 0.000s | 16.6rpm | 11.5A | 13.0V |
| wp_exp2_raise_current_limit | Increase current command limit | 0.44% | 0.0% | 0.154s | 4.7rpm | 18.4A | 13.0V |
| wp_exp3_increase_current_pi | Increase current loop PI (critical for low inductance) | 3.29% | 0.0% | 0.162s | 0.9rpm | 18.0A | 13.0V |
| wp_exp4_increase_speed_pi | Increase speed loop PI gains | 3.29% | 0.0% | 0.162s | 0.9rpm | 18.0A | 13.0V |
| wp_exp5_optimized | Optimized params for water pump | 1.69% | 0.2% | 0.099s | 20.0rpm | 23.2A | 13.0V |
| wp_exp6_auto_design | Auto-designed gains | 1.81% | 0.2% | 0.098s | 30.6rpm | 24.6A | 13.0V |

## Detailed Analysis

### wp_exp0_baseline: Baseline conservative params

**Parameters:**
- speed_Kp: 0.0010, speed_Ki: 0.50
- current_Kp: 0.50, current_Ki: 50.0
- speed_limit (iq cmd): 5.0 A
- voltage_limit: 7.5 V

**Results:**
- Final speed: 2016.3 rpm
- Steady state error: 42.39%
- Speed oscillation: 2.20 rpm
- Rise time: 0.000 s
- Overshoot: 0.0%
- Max current: 8.7 A
- Max voltage: 7.5 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (7.5V > 7.8V limit)
- Large steady error (42.39%)
- Low oscillation (std=2.2rpm)
- Low overshoot (0.0%)

### wp_exp1_raise_voltage: Increase voltage limit

**Parameters:**
- speed_Kp: 0.0010, speed_Ki: 0.50
- current_Kp: 0.50, current_Ki: 50.0
- speed_limit (iq cmd): 5.0 A
- voltage_limit: 13.0 V

**Results:**
- Final speed: 2054.2 rpm
- Steady state error: 41.31%
- Speed oscillation: 16.63 rpm
- Rise time: 0.000 s
- Overshoot: 0.0%
- Max current: 11.5 A
- Max voltage: 13.0 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (13.0V > 7.8V limit)
- Large steady error (41.31%)
- Moderate oscillation (std=16.6rpm)
- Low overshoot (0.0%)

### wp_exp2_raise_current_limit: Increase current command limit

**Parameters:**
- speed_Kp: 0.0010, speed_Ki: 0.50
- current_Kp: 0.50, current_Ki: 50.0
- speed_limit (iq cmd): 12.0 A
- voltage_limit: 13.0 V

**Results:**
- Final speed: 3484.6 rpm
- Steady state error: 0.44%
- Speed oscillation: 4.71 rpm
- Rise time: 0.154 s
- Overshoot: 0.0%
- Max current: 18.4 A
- Max voltage: 13.0 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (13.0V > 7.8V limit)
- Good steady error (0.44%)
- Low oscillation (std=4.7rpm)
- Low overshoot (0.0%)

### wp_exp3_increase_current_pi: Increase current loop PI (critical for low inductance)

**Parameters:**
- speed_Kp: 0.0010, speed_Ki: 0.50
- current_Kp: 2.00, current_Ki: 200.0
- speed_limit (iq cmd): 12.0 A
- voltage_limit: 13.0 V

**Results:**
- Final speed: 3385.0 rpm
- Steady state error: 3.29%
- Speed oscillation: 0.92 rpm
- Rise time: 0.162 s
- Overshoot: 0.0%
- Max current: 18.0 A
- Max voltage: 13.0 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (13.0V > 7.8V limit)
- Moderate steady error (3.29%)
- Low oscillation (std=0.9rpm)
- Low overshoot (0.0%)

### wp_exp4_increase_speed_pi: Increase speed loop PI gains

**Parameters:**
- speed_Kp: 0.0050, speed_Ki: 5.00
- current_Kp: 2.00, current_Ki: 200.0
- speed_limit (iq cmd): 12.0 A
- voltage_limit: 13.0 V

**Results:**
- Final speed: 3385.0 rpm
- Steady state error: 3.29%
- Speed oscillation: 0.92 rpm
- Rise time: 0.162 s
- Overshoot: 0.0%
- Max current: 18.0 A
- Max voltage: 13.0 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (13.0V > 7.8V limit)
- Moderate steady error (3.29%)
- Low oscillation (std=0.9rpm)
- Low overshoot (0.0%)

### wp_exp5_optimized: Optimized params for water pump

**Parameters:**
- speed_Kp: 0.0100, speed_Ki: 10.00
- current_Kp: 3.00, current_Ki: 300.0
- speed_limit (iq cmd): 15.0 A
- voltage_limit: 13.0 V

**Results:**
- Final speed: 3440.7 rpm
- Steady state error: 1.69%
- Speed oscillation: 19.97 rpm
- Rise time: 0.099 s
- Overshoot: 0.2%
- Max current: 23.2 A
- Max voltage: 13.0 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (13.0V > 7.8V limit)
- Moderate steady error (1.69%)
- Moderate oscillation (std=20.0rpm)
- Low overshoot (0.2%)

### wp_exp6_auto_design: Auto-designed gains

**Parameters:**
- speed_Kp: 2.0814, speed_Ki: 628.32
- current_Kp: 2.26, current_Ki: 444.4
- speed_limit (iq cmd): 15.0 A
- voltage_limit: 13.0 V

**Results:**
- Final speed: 3436.6 rpm
- Steady state error: 1.81%
- Speed oscillation: 30.56 rpm
- Rise time: 0.098 s
- Overshoot: 0.2%
- Max current: 24.6 A
- Max voltage: 13.0 V (linear limit: 7.8 V)

**Analysis:**
- **Voltage saturated** (13.0V > 7.8V limit)
- Moderate steady error (1.81%)
- Moderate oscillation (std=30.6rpm)
- Low overshoot (0.2%)

## Conclusion

**Best configuration: wp_exp2_raise_current_limit**

```python
ctrl_params = ControllerParams(
    CL_TS=1e-4,
    VL_TS=1e-3,
    speed_Kp=0.0010,
    speed_Ki=0.50,
    speed_limit=12.0,
    current_Kp=0.50,
    current_Ki=50.0,
    current_limit=13.0,
)
```

Steady state error: **0.44%**
