import numpy as np
from numpy.linalg import inv

# Constraints
start = [1, 1, 0]
end   = [2, 1, 0]
T     = 1

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
print(x)

# Coefficients
s0 = start[0]
s1 = start[1]
s2 = start[2] / 2
s3 = x[0]
s4 = x[1]
s5 = x[2]
coeffs = [s0, s1, s2, s3, s4, s5]

# For evaluating paths
def eval_min_jerk(c, t):
    t2 = t * t
    t3 = t * t2
    t4 = t * t3
    t5 = t * t4
    return c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5

for i in range(100):
    t = float(i) / 100.0
    res = eval_min_jerk(coeffs, t)
    print(res)


