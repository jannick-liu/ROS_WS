#!/usr/bin/env python3
from casadi import *
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import pandas as pd




x  = MX.sym('x', 1)  # state
u  = MX.sym('u', 4)  # control

ns = 40  # number of point
n_obs = 5 # number of obstacles
#n_obs = MX.sym('n_obs',1)



X  = MX.sym('X',1) # Follow the length of the path
dx = X/ns # length between two point
y_star = MX.sym('y_star',1)
k_star = MX.sym('k_star',1)
x_star = MX.sym('x_star',1)
safe_dis = MX.sym('safe_dis',1)
car_width = MX.sym('car_width',1)
global_coeff = MX.sym('global_coeff',4) # global_path_coeff
rt_coeff = MX.sym('rt_coeff',4) # rt_side
lt_coeff = MX.sym('lt_coeff',4) # lt_side
#obs = MX.sym('obs',5)
obs = MX.sym('obs',5*n_obs)



# 
y_local = u[0] * x ** 3 + u[1] * x ** 2 + u[2] * x + u[3]
k = 3 * u[0] * x ** 2 + 2 * u[1] * x + u[2]
k2 = 6 * u[0] * x + 2 * u[1]
K = fabs(k2) / ((1 + k ** 2) ** (3 / 2))
S = sqrt(1 + k ** 2)

f_up_y = y_local + (car_width / 2) * (1 / sqrt(k ** 2 + 1))
f_down_y = y_local - (car_width / 2) * (1 / sqrt(k ** 2 + 1))
f_up_x = x - (((car_width / 2) * k) / sqrt(k ** 2 + 1))
f_down_x = x + (((car_width / 2) * k) / sqrt(k ** 2 + 1))
# Create CasADi functions
y_fn = Function('y_fn', [x, u], [y_local])
k_fn = Function('k_fn', [x, u], [k])
k2_fn = Function('k2_fn', [x, u], [k2])
K_fn = Function('K_fn', [x, u], [K])
S_fn = Function('S_fn', [x, u], [S])
f_up_y_fn = Function('f_up_y_fn', [x, u,car_width], [f_up_y])
f_down_y_fn = Function('f_down_y_fn', [x, u,car_width], [f_down_y])
f_up_x_fn = Function('f_up_x_fn', [x, u,car_width], [f_up_x])
f_down_x_fn = Function('f_down_x_fn', [x, u,car_width], [f_down_x])


# ziel_fun J
coeff_diff = [u[0] - global_coeff[0], 
              u[1] - global_coeff[1], 
              u[2] - global_coeff[2], 
              u[3] - global_coeff[3]]
inter_diff_sq = (pow(coeff_diff[0], 2) / 7) * pow(x, 7) + \
                ((2 * coeff_diff[0] * coeff_diff[1] / 6) * pow(x, 6)) + \
                (((2 * coeff_diff[0] * coeff_diff[2] + pow(coeff_diff[1], 2)) / 5) * pow(x, 5)) + \
                (((2 * coeff_diff[0] * coeff_diff[3] + 2 * coeff_diff[1] * coeff_diff[2]) / 4) * pow(x, 4)) + \
                (((2 * coeff_diff[1] * coeff_diff[3] + pow(coeff_diff[2], 2)) / 3) * pow(x, 3)) + \
                (((2 * coeff_diff[2] * coeff_diff[3]) / 2) * pow(x, 2)) + \
                (pow(coeff_diff[3], 2) * x)
intergral = Function('intergral', [x, u, global_coeff], [inter_diff_sq])
J =(intergral(x_star+X, u, global_coeff) - intergral(x_star, u ,global_coeff))


# constrains
y_lt = lt_coeff[0]*pow(x,3) + lt_coeff[1]*pow(x,2) + lt_coeff[2]*pow(x,1) + lt_coeff[3]*pow(x,0)
y_rt = rt_coeff[0]*pow(x,3) + rt_coeff[1]*pow(x,2) + rt_coeff[2]*pow(x,1) + rt_coeff[3]*pow(x,0)
y_lt_fn = Function('y_lt_fn', [x, lt_coeff], [y_lt])
y_rt_fn = Function('y_rt_fn', [x, rt_coeff], [y_rt])

g_1 = []
g_3 = []
for i in range(ns):
    x_val = x_star + i*dx
    #g_1_up =  -f_up_y_fn(f_up_x_fn(x_val,u,car_width), u, car_width) + y_lt_fn(x_val, lt_coeff) + safe_dis
    #g_1_down =  f_down_y_fn(f_down_x_fn(x_val,u,car_width), u, car_width) - y_rt_fn(x_val, rt_coeff) - safe_dis

    g_1_up =  -f_up_y_fn(f_up_x_fn(x_val,u,car_width), u, car_width) + y_lt_fn(f_up_x_fn(x_val,u,car_width), lt_coeff) + safe_dis
    g_1_down =  f_down_y_fn(f_down_x_fn(x_val,u,car_width), u, car_width) - y_rt_fn(f_down_x_fn(x_val,u,car_width), rt_coeff) - safe_dis
    g_1.extend([g_1_up, g_1_down]) 
    g3 = 2 - K_fn(x_val,u)
    g_3.append(g3)


# obstacles avoidance
#g_2 = []
#for j in range(n_obs):
#    x_obs = obs[0+5*j]
#    y_obs = obs[1+5*j]
#    r_obs = obs[2+5*j]
#    x_vals = casadi.linspace(x_obs - r_obs, x_obs + r_obs, 10)  # 生成 CasADi 符号范围
#    x_vals_list = casadi.vertsplit(x_vals)  # 将符号矩阵拆分为列表
#    for x_val in x_vals_list:
#        g2 = -1+(pow((x_val-obs[0+5*j])*cos(obs[4+5*j]) + (y_fn(x_val,u)-obs[1+5*j])*sin(obs[4+5*j]),2)/pow(obs[2+5*j]+car_width/2 + safe_dis,2) + pow((x_val-obs[0+5*j])*sin(obs[4+5*j]) - (y_fn(x_val,u)-obs[1+5*j])*cos(obs[4+5*j]),2)/pow(obs[3+5*j]+car_width/2 + safe_dis,2))
#        g_2.append(g2)



g_2 = []
for i in range(ns):
    x_val = x_star + i*dx
    for j in range(n_obs):
        x_val = x_star + i*dx
        g2 = -1+(pow((x_val-obs[0+5*j])*cos(obs[4+5*j]) + (y_fn(x_val,u)-obs[1+5*j])*sin(obs[4+5*j]),2)/pow(obs[2+5*j]+car_width/2 + safe_dis,2) + pow((x_val-obs[0+5*j])*sin(obs[4+5*j]) - (y_fn(x_val,u)-obs[1+5*j])*cos(obs[4+5*j]),2)/pow(obs[3+5*j]+car_width/2 + safe_dis,2))
        g_2.append(g2)


g_4 = y_fn(x_star,u) - y_star
g_5 = k_fn(x_star,u) - k_star



# nlp proble definieren
nlp = []
#nlp = {'x':u,  'p': vertcat(X, car_width, global_coeff,  lt_coeff, rt_coeff, obs, safe_dis, x_star, y_star, k_star),'f':J, 'g': vertcat(*g_1,*g_2,*g_3,g_4,g_5)} 
nlp = {'x':u,  'p': vertcat(X, car_width, global_coeff,  lt_coeff, rt_coeff, obs, safe_dis, x_star, y_star, k_star),'f':J, 'g': vertcat(*g_1, *g_2, *g_3,g_4,g_5)} 
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

# solver
solver = nlpsol('solver', 'ipopt', nlp,opts)


compiler = "gcc"    # Linux
flags = ["-O3"] # Linux/OSX

#generate_c = True
generate_c = False
# external c codegen
plugin_name = "geo_local_path_"
if generate_c:
    solver = nlpsol('solver', 'ipopt', nlp,opts);
    solver.generate_dependencies(plugin_name+".c")

    import subprocess
    cmd_args = [compiler,"-fPIC","-shared"]+flags+[plugin_name+".c","-o",plugin_name+".so"]
    subprocess.run(cmd_args)

#solver = nlpsol("solver", "ipopt", plugin_name+".so")
# nlp problem use
u_opt_all = []
u0 = [0, 0, 0, 0]

X_val = 1.5
car_width_val = 1
global_coeff_val = [0,0,0,0]
lt_coeff_val =    [0,0,0,3]
rt_coeff_val =  [0,0,0,-3]
obs_val = np.array([4, 0,  0.5,  0.5 , 0,
                    5,  0,  0.8,  0.8, 0,
                    6, 0,  0.3,  0.3, 0,
                    100, 100, 0.01, 0.01,0,
                    100, 100, 0.01, 0.01,0])

                 
safe_dis_val = 0.1
x_star_val = 0
y_star_val = 0
k_star_val = 0

#if len(obs_val)<n_obs:
#    n_fill = n_obs-len(obs_val)
#    obs_val = np.vstack([obs_val, np.tile([0, 0, 0, 0, 0], (n_fill, 1))])
obs_val_reshape = obs_val.reshape(-1,1)


params = vertcat(X_val, car_width_val, global_coeff_val, lt_coeff_val, rt_coeff_val, obs_val_reshape, safe_dis_val, x_star_val, y_star_val, k_star_val)

lbu = [-inf,-inf,-inf,-inf]
ubu = [inf,inf,inf,inf]
lbg = []
ubg = []

for i in range(3*ns+ns*n_obs):
    lbg.extend([0.0])  
    ubg.extend([np.inf])
lbg.extend([0.0, 0.0]) 
ubg.extend([0.0, 0.0])

sol = solver(x0=u0, p=params, lbx=lbu, ubx=ubu, lbg=lbg, ubg=ubg)
u_opt = sol['x']
u_opt_all.append(u_opt)

# 可视化障碍物
fig, ax = plt.subplots()
x_plot_all = []
f_values_all = []
f_up_y_all = []
f_down_y_all = []
f_up_x_all = []
f_down_x_all = []

# 优化路径绘制
for i in range(8):
    y_star_val = u_opt[0] * x_star_val ** 3 + u_opt[1] * x_star_val ** 2 + u_opt[2] * x_star_val + u_opt[3]
    k_star_val =  3 * u_opt[0] * x_star_val ** 2 + 2 * u_opt[1] * x_star_val + u_opt[2]
    params = vertcat(X_val, car_width_val, global_coeff_val, lt_coeff_val, rt_coeff_val, obs_val_reshape, safe_dis_val, x_star_val, y_star_val, k_star_val)

    sol = solver(x0=u0, p=params, lbx=lbu, ubx=ubu, lbg=lbg, ubg=ubg)
    u_opt = sol['x']
    # 计算轨迹点
    x_max_plot = x_star_val + 1
    x_plot = np.linspace(x_star_val, x_max_plot, 100)
    f_values = []
    f_up_y_values = []
    f_down_y_values = []
    f_up_x_values = []
    f_down_x_values = []
    for x_val in x_plot:
        #f_values.append(y_fn(x_val, u_opt).full().flatten()[0])
        #f_up_y_values.append(f_up_y_fn(x_val, u_opt,car_width).full().flatten()[0])
        #f_down_y_values.append(f_down_y_fn(x_val, u_opt,car_width).full().flatten()[0])
        #f_up_x_values.append(f_up_x_fn(x_val, u_opt,car_width).full().flatten()[0])
        #f_down_x_values.append(f_down_x_fn(x_val, u_opt,car_width).full().flatten()[0])
        f_values.append(float(y_fn(x_val, u_opt).full()[0]))
        f_up_y_values.append(float(f_up_y_fn(x_val, u_opt, car_width_val).full()[0]))
        f_down_y_values.append(float(f_down_y_fn(x_val, u_opt, car_width_val).full()[0]))
        f_up_x_values.append(float(f_up_x_fn(x_val, u_opt, car_width_val).full()[0]))
        f_down_x_values.append(float(f_down_x_fn(x_val, u_opt, car_width_val).full()[0]))

    ax.plot(x_plot, f_values)
    ax.plot(f_up_x_values, f_up_y_values)
    ax.plot(f_down_x_values, f_down_y_values)
    # 记录数据
    x_plot_all.append(x_plot.tolist())
    f_values_all.append(f_values)
    f_up_y_all.append(f_up_y_values)
    f_down_y_all.append(f_down_y_values)
    f_up_x_all.append(f_up_x_values)
    f_down_x_all.append(f_down_x_values)

    u_opt_all.append(u_opt)
    x_star_val = x_star_val + 1

# 创建数据目录（如果不存在）
save_dir = os.path.expanduser("~/ros_ws/date_process/global_path")
os.makedirs(save_dir, exist_ok=True)

# 展平数据以适应 DataFrame
x_values_flat = [x for sublist in x_plot_all for x in sublist]
y_values_flat = [y for sublist in f_values_all for y in sublist]
y_up_values_flat = [y for sublist in f_up_y_all for y in sublist]
y_down_values_flat = [y for sublist in f_down_y_all for y in sublist]
x_up_values_flat = [x for sublist in f_up_x_all for x in sublist]
x_down_values_flat = [x for sublist in f_down_x_all for x in sublist]
# 构造 DataFrame
df = pd.DataFrame({
    "x_val": x_values_flat,
    "y_val": y_values_flat,
    "y_up_val": y_up_values_flat,
    "y_down_val": y_down_values_flat,
    "x_up_val": x_up_values_flat,
    "x_down_val": x_down_values_flat
})
# 保存为 CSV
csv_path = os.path.join(save_dir, "geo.csv")
df.to_csv(csv_path, index=False)
print(f"数据已保存至 {csv_path}")




# 绘制障碍物为圆
for j in range(n_obs):
    x_obs = obs_val[0 + 5 * j]
    y_obs = obs_val[1 + 5 * j]
    r_obs_x = obs_val[2 + 5 * j] 
    r_obs_y = obs_val[3 + 5 * j]

    # 画椭圆的障碍物
    circle = plt.Circle((x_obs, y_obs), max(r_obs_x, r_obs_y), color='green', alpha=0.5, label="Obstacle" if j == 0 else "")
    ax.add_patch(circle)

# 添加标签和图例
ax.axhline(y=3, color='green', linestyle='--', label='y=3')
ax.axhline(y=-3, color='blue', linestyle='--', label='y=-3')
ax.axhline(y=0, color='red', linestyle='--', label='y=0')
ax.set_ylim(-4.5, 4.5)



ax.set_aspect('equal', adjustable='datalim')
ax.grid(True)
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.legend()

# 显示图形
plt.show()

