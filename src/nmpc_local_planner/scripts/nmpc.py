from casadi import *
import numpy as np
import matplotlib.pyplot as plt
import math
import scipy.interpolate as ip

from path_module import *
from opt_module import *
T = 4 # Time horizon
ns = 20 # number of control intervals

# Dimensions
nx = 4
nu = 2
nxu = nx + nu
npar = 2
n_obs = 10

# ode, dynamic model
x  = SX.sym('x', nx)  # state
u  = SX.sym('u', nu)  # control
xICR = SX.sym('xICR',1) # skid-steering parameter

xdot = vertcat(x[3]*cos(x[2]) + xICR * u[1]*sin(x[2]), \
              x[3]*sin(x[2]) - xICR * u[1]*cos(x[2]), \
              u[1], \
              u[0]) # u[0]->acc u[1]->steer
f = Function('f', [x,u,xICR], [xdot])

DT = MX.sym('DT')
X0 = MX.sym('X0',nx)
U  = MX.sym('U',nu)
# define ode and objective function
X_ICR = MX.sym('X_ICR')
Width = MX.sym('Width')
M_param = vertcat(X_ICR,Width)
disc = discretization("rk4",nx,f)
XPred = disc.rk4(X0,U,M_param[0],DT)

F = Function('F', [X0, U, M_param, DT], [XPred]).expand()    #, Y_LT, Y_RT

NV = nx*(ns+1) + nu*ns
V = MX.sym("V", NV)
Vd = reshape(V[:-nx],nxu,ns) # matrix nxu * ns
Xk = Vd[:nx,:]
Uk = Vd[nx:,:] # matrix nu * ns
Xn= F.map(ns, 'openmp',True)(Xk, Uk, M_param, DT)

# project x coord into reference curve
x_offset = SX.sym('x_offset')
ct_coeff = SX.sym('ct_coeff',4)

y_ct = ct_coeff[3]*pow(x[0]+x_offset,3) + ct_coeff[2]*pow(x[0]+x_offset,2) + ct_coeff[1]*pow(x[0]+x_offset,1) + ct_coeff[0]*pow(x[0]+x_offset,0)

# project x coord into left and right curve
x_offset_l = SX.sym('x_offset_l')
x_offset_r = SX.sym('x_offset_r')
rt_coeff = SX.sym('rt_coeff',4)
lt_coeff = SX.sym('lt_coeff',4)
width = SX.sym('width')

y_lt = lt_coeff[3]*pow(x[0]+x_offset_l,3) + lt_coeff[2]*pow(x[0]+x_offset_l,2) + lt_coeff[1]*pow(x[0]+x_offset_l,1) + lt_coeff[0]*pow(x[0]+x_offset_l,0)
y_rt = rt_coeff[3]*pow(x[0]+x_offset_r,3) + rt_coeff[2]*pow(x[0]+x_offset_r,2) + rt_coeff[1]*pow(x[0]+x_offset_r,1) + rt_coeff[0]*pow(x[0]+x_offset_r,0)
g_1 = x[1] + width - y_lt
g_2 = y_rt + width - x[1]

# obstacle parameters
obs = SX.sym('obs',5) # x, y, r1, r2, theta
obs_dist = pow((x[0]-obs[0])*cos(obs[4]) + (x[1]-obs[1])*sin(obs[4]),2)/pow(obs[2],2) + pow((x[0]-obs[0])*sin(obs[4]) - (x[1]-obs[1])*cos(obs[4]),2)/pow(obs[3],2)
g_3 = 1 - obs_dist

v_ref = SX.sym('vref')
x_tf = SX.sym('xtf',nx)
# weighting parameters
w_x = SX.sym('w_x',nx)
w_u = SX.sym('w_u',nu)
w_y = SX.sym('w_y')
w_v = SX.sym('w_v')
w_tf = SX.sym('w_tf',nx)

# reference following
follow_term = w_y*pow((x[1] - y_ct),2) + w_v*pow((x[3] - v_ref),2) + mtimes(u.T,mtimes(diag(w_u),u))
terminal_term = mtimes((x-x_tf).T,mtimes(diag(w_tf),(x-x_tf)))

obj_follow = Function('obj_follow', [x,u, v_ref, ct_coeff, x_offset, vertcat(w_y,w_v,w_u)],[follow_term])
obj_terminal = Function('obj_terminal',[x,x_tf,w_tf],[terminal_term])

w_obs = SX.sym('w_obs')
w_y_l = SX.sym('w_y_l')
w_y_r = SX.sym('w_y_r')

left_side_term = w_y_l*exp(g_1) # penalty term for left and right distance
right_side_term = w_y_r*exp(g_2)

obs_term = w_obs*exp(g_3) # penalty term for obstacle avoidance

obj_side_l = Function('obj_side_l',[x,lt_coeff,w_y_l,x_offset_l,width],[left_side_term])
obj_side_r = Function('obj_side_r',[x,rt_coeff,w_y_r,x_offset_r,width],[right_side_term])
obj_obs = Function('obj_obs',[x,obs,w_obs],[obs_term])

Q = 0

XF = MX.sym('XF',nx)
V_ref = MX.sym('V_ref')
CT_coeff = MX.sym("CT",4)
X_offset_c = MX.sym('X_offset_c')
W_ref = MX.sym('W_ref',4)
Q += obj_follow(XF,U,V_ref,CT_coeff,X_offset_c,W_ref)

LT_coeff = MX.sym("LT",4)
W_lt = MX.sym('W_lt')
X_offset_l = MX.sym('X_offset_l')
RT_coeff = MX.sym("RT",4)
W_rt = MX.sym('W_rt')
X_offset_r = MX.sym('X_offset_r')
Q += obj_side_l(XF,LT_coeff,W_lt,X_offset_l,Width) + obj_side_r(XF,RT_coeff,W_rt,X_offset_r,Width) 

Obs_ml = MX.sym("Obs_ml",5,n_obs)
W_obs = MX.sym('W_obs',n_obs)
obs_term_ml = obj_obs.map(n_obs, 'openmp',True)(XF,Obs_ml,W_obs)
Q_obs = sum2(obs_term_ml)
Q += Q_obs

Weights = vertcat(W_ref,W_lt,W_rt,W_obs)
X_offset = veccat(X_offset_c,X_offset_l,X_offset_r)
Path_coeff = vertcat(CT_coeff,LT_coeff,RT_coeff)

QF = Function('QF', [XF, U, Width, V_ref, Path_coeff,X_offset,Obs_ml,Weights], [Q]).expand()    #, Y_LT, Y_RT
Qn= QF.map(ns, 'openmp',True)(Xn, Uk, Width, V_ref, Path_coeff, X_offset, Obs_ml, Weights)

X_tf = MX.sym('X_tf',nx)
W_tf = MX.sym('W_tf',nx)
Q_terminal = obj_terminal(V[-nx:],X_tf,W_tf)

J = sum2(Qn) + Q_terminal
W_cost = vertcat(Weights,W_tf)

# continuity constraints
gaps = Xn[:,:]-horzcat(Xk[:,1:],V[-nx:])

nlp = []
nlp = {'x':V,  'p': veccat(M_param, DT, V_ref, Path_coeff, X_offset, X_tf, Obs_ml, W_cost),'f':J,'g': vec(gaps)} #,vec(g_lt),vec(g_rt)

opts = {}
ipopt_opts = {}
ipopt_opts["tol"] = 1e-5;
ipopt_opts["max_iter"] = 100;
ipopt_opts["print_level"] = 5;
ipopt_opts["sb"] = "yes";
ipopt_opts["acceptable_tol"] = 1e-5;
ipopt_opts["acceptable_iter"] = 0;
ipopt_opts["linear_solver"] = 'ma27';
# ipopt_opts["hessian_approximation"] = "limited-memory";
ipopt_opts["warm_start_init_point"] = "yes";
ipopt_opts["warm_start_bound_push"] = 1e-6;
ipopt_opts["warm_start_mult_bound_push"] = 1e-6;
opts["expand"] = False
opts["print_time"] = 0;
opts["ipopt"] = ipopt_opts

compiler = "gcc"    # Linux
flags = ["-O3"] # Linux/OSX

generate_c = True
# external c codegen
plugin_name = "nmpc_cartesian_dyn_whizzy"
if generate_c:
    solver = nlpsol('solver', 'ipopt', nlp,opts);
    solver.generate_dependencies(plugin_name+".c")

    import subprocess
    cmd_args = [compiler,"-fPIC","-shared"]+flags+[plugin_name+".c","-o",plugin_name+".so"]
    subprocess.run(cmd_args)

