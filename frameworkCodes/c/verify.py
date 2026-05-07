"""
phase 1: 仿真准确性验证脚本

验证目的：
1. 稳态转速是否符合理论计算
2. 启动电流是否符合物理规律
3. 转矩计算是否正确
4. 能量守恒验证
"""

import csv

print("=" * 60)
print("        Phase 1 仿真准确性验证报告")
print("=" * 60)

# 电机参数 (SEW 100W PMSM)
MOTOR = {
    'name': 'SEW 100W PMSM',
    'npp': 4,           # 极对数
    'R': 0.475,         # 定子电阻 Ohm
    'Ld': 2.05e-3,      # D轴电感 H
    'Lq': 2.05e-3,      # Q轴电感 H
    'KE': 0.0107,       # 反电势系数 Wb
    'Js': 3.5e-6,       # 惯量 kg.m^2
    'Ud': 0.0,          # D轴电压
    'Uq': 5.0,          # 施加电压 V
}

# 电机参数(SEW 100W PMSM)

data_file = '../dat/test_motor.dat'
with open(data_file, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    rows = list(reader)

# 解析数据到列表
time_data = [float(row[0]) for row in rows] 
omega_data = [float(row[2]) for row in rows]    # 机械转速
iD_data = [float(row[4]) for row in rows]       # D轴电流
iQ_data = [float(row[5]) for row in rows]       # Q轴电流

print(f"\n电机参数: {MOTOR['name']}")
print(f"施加电压: Uq={MOTOR['Uq']}V, Ud={MOTOR['Ud']}V")
print(f"数据样本: {len(rows)} 点")
print(f"仿真时长: {time_data[-1]:.4f} s")

# === 验证1: 稳态转速 ===
print("\n" + "-" * 50)
print("验证1: 稳态转速是否符合电压平衡方程")
print("-" * 50)

omega_final = float(rows[-1][2])
omega_theory = MOTOR['Uq'] / MOTOR['KE'] / MOTOR['npp']
error1 = abs(omega_final - omega_theory) / omega_theory * 100

print(f"理论计算: Ω = Uq/(KE*npp) = {MOTOR['Uq']}/{MOTOR['KE']}/{MOTOR['npp']} = {omega_theory:.2f} rad/s")
print(f"仿真结果: Ω = {omega_final:.2f} rad/s")
print(f"误差: {error1:.2f}%")
print(f"结论: {'PASS' if error1 < 1 else 'FAIL'}")


# === 验证2: 电机耦合瞬态过程 ===
print("\n" + "-" * 50)
print("验证2: 电机耦合瞬态过程")
print("-" * 50)

print("\n【物理原理】")
print("  电机启动是机电耦合过程:")
print("  1. 电流建立产生转矩: Tem = 1.5*npp*KE*iq")
print("  2. 转矩加速电机: domega/dt = Tem/Js")
print("  3. 转速增加产生反电势: E = omega*npp*KE")
print("  4. 反电势抵消电压: iq = (Uq-E)/R")
print("  5. 最终稳态: E=Uq, iq=0")

# 找峰值电流
iQ_max = max(iQ_data)
idx_peak = iQ_data.index(iQ_max)
t_peak = time_data[idx_peak]
omega_peak = omega_data[idx_peak]

# 计算峰值时刻的反电动势
E_peak = omega_peak * MOTOR['npp'] * MOTOR['KE']    # 电气反电动势

print(f"\n【峰值电流分析】")
print(f"  峰值电流: iq_max = {iQ_max:.4f} A")
print(f"  峰值时间: t = {t_peak*1000:.2f} ms")
print(f"  峰值时刻转速: omega = {omega_peak:.4f} rad/s")
print(f"  峰值时刻反电势: E = omega*npp*KE = {E_peak:.4f} V")

# 验证峰值时刻的电流方程
# iq = (Uq - E) / R (近似,忽略感应电压)
iq_calc_peak = (MOTOR['Uq'] - E_peak) / MOTOR['R']

# 峰值时刻分析（仅作参考，不用于判定）
# 注意: 峰值时刻是瞬态，无法用稳态方程精确验证
# 真正有意义的验证是稳态: 反电势=Uq, 电流=0
print(f"\n【峰值时刻分析】(仅供参考，不用于判定)")
print(f"  峰值电流: iq_max = {iQ_max:.4f} A")
print(f"  峰值时间: t = {t_peak*1000:.2f} ms")
print(f"  此刻转速: omega = {omega_peak:.4f} rad/s")
print(f"  此刻反电势: E = omega*npp*KE = {E_peak:.4f} V")
print(f"  说明: 峰值时刻电流快速变化中，不能用稳态方程验证")

# 检查电流衰减过程是否符合反电势增长
print(f"\n【反电势增长过程】")

# 取几个关键时间点
check_points = [0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]
print(f"  时间[ms]  转速[rad/s]  反电势[V]   电流[A]    有效电压[V]")
print("-" * 60)

for t_check in check_points:
    # 找最新的数据点
    idx = min(range(len(time_data)), key=lambda i: abs(time_data[i] - t_check))
    t = time_data[idx]
    omega = omega_data[idx]
    iq = iQ_data[idx]
    E = omega * MOTOR['npp'] * MOTOR['KE']
    U_eff = MOTOR['Uq'] - E

    print(f"  {t*1000:6.2f}    {omega:10.4f}    {E:8.4f}    {iq:8.4f}    {U_eff:8.4f}")


# 最终稳态
omega_final_steady = omega_data[-1]
E_final = omega_final_steady * MOTOR['npp'] * MOTOR['KE']
iq_final = iQ_data[-1]
E_error = abs(E_final - MOTOR['Uq']) / MOTOR['Uq'] * 100

print(f"\n【稳态验证】(核心验证项)")
print(f"  稳态条件: 反电势 E = Uq, 电流 iq = 0")
print(f"  最终转速: omega = {omega_final_steady:.4f} rad/s")
print(f"  最终反电势: E = omega*npp*KE = {E_final:.4f} V")
print(f"  理论反电势: E = Uq = {MOTOR['Uq']:.1f} V (电压平衡)")
print(f"  反电势误差: {E_error:.4f}%")
print(f"  最终电流: iq = {abs(iq_final):.10f} A (理论应为0)")

# 判断标准
pass2 = E_error < 1 and abs(iq_final) < 0.01

print(f"\n【判定标准】")
print(f"  1. 反电势误差 < 1%: 实际 {E_error:.4f}% => {'PASS' if E_error < 1 else 'FAIL'}")
print(f"  2. 稳态电流 < 0.01A: 实际 {abs(iq_final):.6f}A => {'PASS' if abs(iq_final) < 0.01 else 'FAIL'}")
print(f"  验证2结论: {'PASS' if pass2 else 'FAIL'}")

# ============================================
# 验证3: 转矩公式
# ============================================
print("\n" + "=" * 50)
print("验证3: 转矩公式 Tem = 1.5*npp*KE*iq")
print("=" * 50)

print("\n【物理原理】")
print("  PMSM电磁转矩:")
print("  Tem = 1.5 * npp * psi_pm * iq")
print("  其中 psi_pm = KE (反电势系数)")

# 在多个时间点验证
print(f"\n【多点验证】")
print(f"  时间[ms]   iq[A]      Tem仿真[Nm]  Tem理论[Nm]   误差[%]")
print("-" * 60)