# type: ignore
import sys, io, os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from super_config import SuperConfig
import pandas as pd

BASE = os.path.join(os.path.dirname(__file__), "..")
SDAT = os.path.join(BASE, "frameworkCodes", "dat")

def foc_run(steps, cmd_iq, tload, tag):
    cfg = SuperConfig("SEW100W")
    cfg.load_motor_library()
    cfg.load_user_config()
    p = cfg.get_motor_params("SEW100W")
    cfg.user_config["simulation"]["sim.CL_TS"] = 5e-5
    cfg.user_config["simulation"]["sim.NUMBER_OF_STEPS"] = steps
    cfg.user_config["simulation"]["sim.MODE_SELECT"] = 3
    cfg.user_config["test"]["test.cmd_iQ"] = cmd_iq
    cfg.user_config["test"]["test.TLoad"] = tload
    for k in ["npp","IN","R","Ld","Lq","KE","Js","Vdc"]:
        cfg.user_config["motor"][f"motor.{k}"] = p[k]
    cfg.update_super_config()
    exe = os.path.join(BASE, "frameworkCodes", "c", "foc_test.exe")
    if os.path.exists(exe):
        os.remove(exe)
    cfg.compile_simulation("foc_test")
    cfg.run_simulation("foc_test")
    return pd.read_csv(os.path.join(SDAT, "test_motor_foc.dat"))

print("="*50)
print("Test 1: FOC no-load cmd_iQ=2A")
print("="*50)
df1 = foc_run(steps=2000, cmd_iq=2.0, tload=0.0, tag="no_load")
s1 = df1.iloc[-30:].mean()
print(f"steady omega={s1['varOmega']:.1f}, iQ={s1['iQ']:.4f}, Tem={s1['Tem']:.4f}")

print("\n"+"="*50)
print("Test 2: FOC TLoad=0.05Nm cmd_iQ=2A")
print("="*50)
iq_th = 0.05/(1.5*4*0.0107)
df2 = foc_run(steps=2000, cmd_iq=2.0, tload=0.05, tag="tload005")
s2 = df2.iloc[-30:].mean()
print(f"steady omega={s2['varOmega']:.1f}, iQ={s2['iQ']:.4f}, Tem={s2['Tem']:.4f}")
print(f"theory iQ={iq_th:.4f}")

print("\n"+"="*50)
print("Test 3: FOC TLoad=0.15Nm cmd_iQ=2A")
print("="*50)
iq_th3 = 0.15/(1.5*4*0.0107)
df3 = foc_run(steps=2000, cmd_iq=2.0, tload=0.15, tag="tload015")
s3 = df3.iloc[-30:].mean()
print(f"steady omega={s3['varOmega']:.1f}, iQ={s3['iQ']:.4f}, Tem={s3['Tem']:.4f}")
print(f"theory iQ={iq_th3:.4f}")
