# type: ignore
"""
foc current mode test
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from super_config import SuperConfig


def run_foc(motor, cmd_iq, tload=0.0, cl_ts=5e-5, steps=200, target="foct_test"):
    cfg = SuperConfig(motor)
    cfg.load_motor_library()
    cfg.load_user_config()
    p = cfg.get_motor_params(motor)
    cfg.user_config["simulation"]["sim.CL_TS"] = cl_ts
    cfg.user_config["simulation"]["sim.NUMBER_OF_STEPS"] = steps
    cfg.user_config["simulation"]["sim.MODE_SELECT"] = 3
    cfg.user_config["test"]["test.cmd_speed"] = 0.0
    cfg.user_config["test"]["test.cmd_iQ"] = cmd_iq
    cfg.user_config["test"]["test.cmd_uD"] = 0.0
    cfg.user_config["test"]["test.cmd_uQ"] = 0.0
    cfg.user_config["test"]["test.TLoad"] = tload
    cfg.user_config["motor"]["motor.npp"] = p["npp"]
    cfg.user_config["motor"]["motor.IN"] = p["IN"]
    cfg.user_config["motor"]["motor.R"] = p["R"]
    cfg.user_config["motor"]["motor.Ld"] = p["Ld"]
    cfg.user_config["motor"]["motor.Lq"] = p["Lq"]
    cfg.user_config["motor"]["motor.KE"] = p["KE"]
    cfg.user_config["motor"]["motor.Js"] = p["Js"]
    cfg.user_config["motor"]["motor.Vdc"] = p["Vdc"]
    cfg.update_super_config()
    cfg.compile_simulation(target)
    cfg.run_simulation(target)
    return pd.read_csv(os.path.join(SDAT, "test_motor_foc.dat"))

print("=" * 70)
print("test 1: foc current tracing (cmd_iQ = 1A, 200 steps = 10ms)")
print("=" * 70)

df1 = run_foc("SEW100W", cmd_iq=1.0, steps=200)
t90 = df1[df1["iQ"] >= 0.9]["time"].min()
overshoot = df1["iQ"].max()