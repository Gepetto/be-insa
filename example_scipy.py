import numpy as np
from scipy.optimize import fmin_bfgs, fmin_slsqp

def cost(x):
     '''Cost f(x,y) = x^2 + 2y^2 - 2xy - 2x '''
     x0 = x[0]
     x1 = x[1]
     return -1*(2*x0*x1 + 2*x0 - x0**2 - 2*x1**2)

def constraint_eq(x):
     ''' Constraint x^3 = y '''
     return np.array([ x[0]**3-x[1] ])

def constraint_ineq(x):
     '''Constraint x>=2, y>=2'''
     return np.array([ x[0]-2,x[1]-2 ])

class CallbackLogger:
     def __init__(self):
          self.nfeval = 1
     def __call__(self,x):
          print('===CBK=== {0:4d}   {1: 3.6f}   {2: 3.6f}    {3: 3.6f}'.format(self.nfeval, x[0], x[1], cost(x)))
          self.nfeval += 1

x0 = np.array([0.0,0.0])
# Optimize cost without any constraints in BFGS, with traces.
xopt_bfgs = fmin_bfgs(cost, x0, callback=CallbackLogger())
print('\n *** Xopt in BFGS = ',xopt_bfgs,'\n\n\n\n')

# Optimize cost without any constraints in CLSQ
xopt_lsq = fmin_slsqp(cost,[-1.0,1.0], iprint=2, full_output=1)
print('\n *** Xopt in LSQ = ',xopt_lsq,'\n\n\n\n')

# Optimize cost with equality and inequality constraints in CLSQ
xopt_clsq = fmin_slsqp(cost,[-1.0,1.0],
                       f_eqcons=constraint_eq, f_ieqcons=constraint_ineq,
                       iprint=2, full_output=1)
print('\n *** Xopt in c-lsq = ',xopt_clsq,'\n\n\n\n')

