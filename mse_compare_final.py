import pandas as pd
import os

folder = "logs"
files = ["routine_pid1.csv", "routine_nc1.csv", "routine_ss1.csv"]

results = []
for f in files:
    path = os.path.join(folder, f)
    df = pd.read_csv(path)
    mse_v = ((df['v'] - df['v_ref']) ** 2).mean()
    mse_w = ((df['w'] - df['w_ref']) ** 2).mean()
    total_mse = mse_v + mse_w
    results.append((f, mse_v, mse_w, total_mse))

results.sort(key=lambda x: x[3])
print("Arquivo que melhor segue a referência:")
print(f"  {results[0][0]} (MSE v: {results[0][1]:.6f}, MSE w: {results[0][2]:.6f}, Total: {results[0][3]:.6f})")

print("\nResumo dos três arquivos:")
for r in results:
    print(f"{r[0]}: MSE v = {r[1]:.6f}, MSE w = {r[2]:.6f}, Total = {r[3]:.6f}")
