import csv, os

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
RESULT_DIR = os.path.join(PROJECT_ROOT, "dat", "bw_experiment")

cases = ['1a_bw5', '1b_bw10', '1c_bw20', '2b_bw150']

for case in cases:
    with open(os.path.join(RESULT_DIR, f'bw_exp_{case}.dat')) as f:
        rows = list(csv.DictReader(f))
    print(f'=== {case} ===')
    indices = [0, 10, 20, 50, 100, 200, 500, 1000, 2000, 4000, 6000, 7999]
    for i in indices:
        if i < len(rows):
            r = rows[i]
            t_ms = float(r['time']) * 1000
            omega = float(r['varOmega'])
            iq = float(r['iQ'])
            cmd_iq = float(r['cmd_iQ'])
            print(f'  t={t_ms:7.1f}ms  omega={omega:8.2f}  iq={iq:6.3f}  cmd_iQ={cmd_iq:8.4f}')
    print()