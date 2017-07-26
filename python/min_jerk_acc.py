import numpy as np
from numpy.linalg import inv


# Problem constraints
time_step = 2
start_vel = 1
end_vel   = 2
start_pos = 1
end_pos   = start_pos + 0.5 * time_step * (start_vel + end_vel)
start_acc = 0
end_acc   = 0

# Programmatic constraints
start = [start_pos, start_vel, start_acc]
end   = [end_pos,   end_vel,   end_acc]
T     = time_step

# Solve Ax=b for x
A = [ [T**3,           T**4,       T**5],
      [3 * T**2,   4 * T**3,   5 * T**4],
      [6 * T,     12 * T**2,  20 * T**3] ]
b = [ end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T**2),
      end[1] - (start[1] + start[2] * T),
      end[2] - start[2] ]

A = np.array(A)
b = np.array(b)
x = inv(A).dot(b)

# Coefficients
s0 = start[0]
s1 = start[1]
s2 = start[2] / 2
s3 = x[0]
s4 = x[1]
s5 = x[2]
coeffs = [s0, s1, s2, s3, s4, s5]
print(coeffs)

def eval_min_jerk(c, t):
    t2 = t * t
    t3 = t * t2
    t4 = t * t3
    t5 = t * t4
    p = c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5
    return p

def calc_deriv(val, dt=0.01):
    res = []
    for i in range(len(val)-1):
        v = (val[i+1] - val[i]) / dt
        res.append(v)
    return res

pos = []
for i in range(200):
    t = float(i) / 100.0
    p = eval_min_jerk(coeffs, t)
    pos.append(p)
print("\nPosition")
print(pos)

vel = calc_deriv(pos)
print("\nVelocity")
print(vel)

acc = calc_deriv(vel)
print("\nAcceleration")
print(acc)
