import numpy as np
from numpy.linalg import inv


# Problem constraints
time_step = 2
start_vel = 1
end_vel   = 2
start_pos = 1
end_pos   = start_pos + 0.5 * time_step * (start_vel + end_vel)

# Programmatic constraints
start = [start_pos, start_vel]
end   = [end_pos,   end_vel]
T     = time_step

# Solve Ax=b for x
A = [ [    T**2,     T**3],
      [2 * T,    3 * T**2] ]
b = [ end[0] - (start[0] + start[1] * T),
      end[1] - start[1] ]
A = np.array(A)
b = np.array(b)
x = inv(A).dot(b)

# Coefficients
s0 = start[0]
s1 = start[1]
s2 = x[0]
s3 = x[1]
coeffs = [s0, s1, s2, s3]
print(coeffs)

# For evaluating paths
def eval_min_jerk(c, t):
    t2 = t * t
    t3 = t * t2
    p = c[0] + c[1]*t + c[2]*t2 + c[3]*t3
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
