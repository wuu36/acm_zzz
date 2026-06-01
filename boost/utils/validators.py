D_MAX = 0.90

def validate_params(Vin, R, L, C, f, D, Rl, Vd):
    errors = []
    warnings = []

    if Vin <= 0:
        errors.append("输入电压Vin必须大于0")
    if R <= 0:
        errors.append("负载电阻R必须大于0")
    if L <= 0:
        errors.append("电感L必须大于0")
    if C <= 0:
        errors.append("电容C必须大于0")
    if f <= 0:
        errors.append("开关频率f必须大于0")
        
    if D is not None:
        if D > D_MAX:
            errors.append(f"占空比D={D:.2f}超过安全上限{D_MAX}, 增益将发散")
        if D < 0.01:
            warnings.append("占空比D极小，输出电压接近输入电压")
    
    if Rl > R * 0.5:
        warnings.append(f"电感内阻RL={Rl:.3f}接近负载内阻R={R:.1f},效率将极低")
    
    return {"errors": errors, "warnings": warnings}

def validate_closed_loop(Vin, Vout_target):
    errors = []
    warnings = []
    
    if Vout_target <= Vin:
        errors.append(f"目标Vout={Vout_target:.1f}V <= Vin={Vin:.1f}V,boost电路无法降压")
    if Vout_target / Vin > 10:
        warnings.append(f"增益比Vout/Vin={Vout_target/Vin:.1f}极高，实际效率可能很低")
        
    D_needed = 1 - Vin/Vout_target
    if D_needed > D_MAX:
        errors.append(f"所需占空比D={D_needed:.2f} > {D_MAX}, 无法实现目标电压")
        
    return {"errors": errors, "warnings": warnings}
    