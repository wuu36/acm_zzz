
import os
import pandas as pd

def read_data(filepath):
    if not os.path.exists(filepath):
        print(f"[cplot] file not found: {filepath}")
        return None
    
    try:
        df = pd.read_csv(filepath)
        # print(df)
        return df
    except Exception as e:
        print(f"[cplot] error reading file: {e}")
        return None
    