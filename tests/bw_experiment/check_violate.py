import csv, os

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
RESULT_DIR = os.path.join(PROJECT_ROOT, "dat", "bw_experiment")

print("=== 2a_bw100 (violate: BW_speed=100) - first 100 steps detail ===")
with open(os.path.join(RESULT_DIR, 'bw_exp_2a_bw100.dat')) as f:
    rows = list(csv.DictReader(f))
for i in range(0, 100, 5):
    r = rows[i]
    t_ms = float(r['time']) * 1000
    omega = float(r['varOmega'])
    iq = float(r['iQ'])
    cmd_iq = float(r['cmd_iQ'])
    print(f'  t={t_ms:7.2f}ms  omega={omega:8.2f}  iq={iq:6.3f}  cmd_iQ={cmd_iq:8.4f}')

print("\n=== 2b_bw150 (violate: BW_speed=150) - first 100 steps detail ===")
with open(os.path.join(RESULT_DIR, 'bw_exp_2b_bw150.dat')) as f:
    rows = list(csv.DictReader(f))
for i in range(0, 100, 5):
    r = rows[i]
    t_ms = float(r['time']) * 1000
    omega = float(r['varOmega'])
    iq = float(r['iQ'])
    cmd_iq = float(r['cmd_iQ'])
    print(f'  t={t_ms:7.2f}ms  omega={omega:8.2f}  iq={iq:6.3f}  cmd_iQ={cmd_iq:8.4f}')