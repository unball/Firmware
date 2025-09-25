import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete, place_poles

# === Saturation limits ===
u_min = -69
u_max = 69

# === Physical parameters ===
R = 0.0215    # wheel radius [m]
L = 0.0825    # wheelbase [m]
T = 0.02      # sampling period [s]

# === Continuous-time model ===
alpha = 1.0

A = np.array([[-alpha, 0],
              [0, -alpha]])

B = np.array([[R/2, R/2],
              [-R/L, R/L]])

# Discretize with ZOH
sys_d = cont2discrete((A, B, np.eye(2), np.zeros((2,2))), T, method='zoh')
G, H, _, _, _ = sys_d

print("Discrete-time state-space equations:")
print("G =")
print(G)
print("H =")
print(H)

# === Simulation setup ===
Nsteps = 100
time = np.arange(Nsteps) * T
v_ref = 0.105 * np.ones(Nsteps)
w_ref = 0.2 * np.ones(Nsteps)
r = np.vstack((v_ref, w_ref))   # shape (2, Nsteps)

# === Define desired poles ===
poles = [0.97, 0.97]   # You can change this

# === Compute K and N ===
pp = place_poles(G, H, poles)
K = pp.gain_matrix

C = np.eye(2)
N = np.linalg.inv(C @ np.linalg.inv(np.eye(2) - G + H @ K) @ H)

print("State feedback gain K =")
print(K)
print("Reference gain N =")
print(N)

# === Initialize state and control ===
x = np.zeros((2, Nsteps))
u = np.zeros((2, Nsteps))

# === Simulate closed-loop response ===
for k in range(Nsteps-1):
    # Compute control
    u_unsat = -K @ x[:, k] + N @ r[:, k]

    # Apply saturation element-wise
    u[:, k] = np.clip(u_unsat, u_min, u_max)

    # Update state
    x[:, k+1] = G @ x[:, k] + H @ u[:, k]

# === Plot 1: States v and w ===
plt.figure()
plt.plot(time, x[0, :], '-', linewidth=1.5, label='v')
plt.plot(time, x[1, :], '--', linewidth=1.5, label='w')
plt.plot(time, v_ref, 'k-.', linewidth=1.2, label='v_ref')
plt.plot(time, w_ref, 'k:', linewidth=1.2, label='w_ref')
plt.xlabel("Time [s]")
plt.ylabel("State values")
plt.title("Step Response: v and w")
plt.legend(loc="lower right")
plt.grid(True)

# === Plot 2: Control inputs u_L and u_R ===
plt.figure()
plt.plot(time, u[0, :], '-', linewidth=1.5, label='u_L')
plt.plot(time, u[1, :], '--', linewidth=1.5, label='u_R')
plt.xlabel("Time [s]")
plt.ylabel("Control input (rad/s)")
plt.title("Control Inputs: ω_L and ω_R")
plt.legend(loc="best")
plt.grid(True)

plt.show()
