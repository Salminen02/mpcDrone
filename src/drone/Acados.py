import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import casadi as ca

# --- 1. Asetukset ---
N  = 200
dt = 0.01

# Kopterin fyysiset vakiot
m, g_accel, L = 0.5, 9.81, 0.3
Ix, Iy, Iz    = 0.0075, 0.0075, 0.03
k_m           = 0.25

# --- 2. AcadosModel ---
model = AcadosModel()
model.name = 'quadrotor_mpcc'

# Tilamuuttujat: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r, theta_path]
x_sym = ca.SX.sym('x', 13)
# Ohjausmuuttujat: [T1,T2,T3,T4, v_theta]
u_sym = ca.SX.sym('u', 5)
# Parametrit: [yaw_world, mu, Cl, ball_cx, ball_cy, ball_cz, ball_r]
p_sym = ca.SX.sym('p', 7)

model.x = x_sym
model.u = u_sym
model.p = p_sym

# --- 3. Rata-apufunktiot ---
def eight_path(theta, yaw):
    a = 5.0
    px_w = a * ca.sin(theta)
    py_w = a * ca.sin(theta) * ca.cos(theta)
    pz_w = 0.0

    tx_w = a * ca.cos(theta)
    ty_w = a * ca.cos(2 * theta)

    cy, sy = ca.cos(yaw), ca.sin(yaw)
    px =  cy * px_w + sy * py_w
    py = -sy * px_w + cy * py_w
    pz = pz_w
    tx =  cy * tx_w + sy * ty_w
    ty = -sy * tx_w + cy * ty_w
    p  = ca.vertcat(px, py, pz)

    t  = ca.vertcat(tx, ty, 0.0)
    t_norm = t / ca.norm_2(t)
    return p, t_norm

def circle_path(theta, yaw):
    a = 5.0
    b = 10.0
    
    px_w = a * ca.sin(theta)
    py_w = b * ca.cos(theta)
    pz_w = 0.0

    tx_w = a * ca.cos(theta)
    ty_w = b * -ca.sin(theta)

    cy, sy = ca.cos(yaw), ca.sin(yaw)
    px =  cy * px_w + sy * py_w
    py = -sy * px_w + cy * py_w
    pz = pz_w
    tx =  cy * tx_w + sy * ty_w
    ty = -sy * tx_w + cy * ty_w
    p  = ca.vertcat(px, py, pz)

    t  = ca.vertcat(tx, ty, 0.0)
    t_norm = t / ca.norm_2(t)
    return p, t_norm

def get_lag_contour(p_ref, t_norm, p_drone):
    err        = p_drone - p_ref
    lag_error  = ca.dot(t_norm, err)
    contour    = ca.sumsqr(err) - lag_error**2
    return lag_error, contour

# --- 4. Dynamiikka (jatkuva-aikainen, acados integroi itse) ---
def quadrotor_ode(x, u, yaw):
    vel   = x[3:6]
    euler = x[6:9]
    rates = x[9:12]

    T_total = ca.sum1(u[0:4])
    tau_x = (u[0] + u[3] - u[1] - u[2]) * L
    tau_y = (u[0] + u[1] - u[2] - u[3]) * L
    tau_z = (u[1] + u[3] - u[0] - u[2]) * k_m
    v_theta = u[4]

    phi, theta, psi = euler[0], euler[1], euler[2]

    ax = (T_total/m)*(ca.cos(psi)*ca.sin(theta)*ca.cos(phi) + ca.sin(psi)*ca.sin(phi))
    ay = (T_total/m)*(ca.sin(psi)*ca.sin(theta)*ca.cos(phi) - ca.cos(psi)*ca.sin(phi))
    az = (ca.cos(phi)*ca.cos(theta)*T_total)/m - g_accel

    return ca.vertcat(
        vel[0], vel[1], vel[2],
        ax, ay, az,
        rates[0], rates[1], rates[2],
        tau_x/Ix, tau_y/Iy, tau_z/Iz,
        v_theta
    )

yaw_world = p_sym[0]
f_expl = quadrotor_ode(x_sym, u_sym, yaw_world)
model.f_expl_expr = f_expl

# --- 5. Kustannusfunktio (EXTERNAL / nonlinear least squares) ---
mu = p_sym[1]    # online-parametri, oletusarvo asetetaan parameter_values:ssa
Cl = p_sym[2]
Cc = 10
Cv = 0.75
Cr = 5

theta_path = x_sym[12]
p_path, t_norm = circle_path(theta_path, yaw_world)
lag_err, contour = get_lag_contour(p_path, t_norm, x_sym[0:3])

# acados EXTERNAL cost: skalaari-lauseke
stage_cost = (Cl * lag_err**2
            + Cc * contour
            - mu * u_sym[4]
            + Cv * ca.sumsqr(x_sym[3:6])
            + Cr * ca.sumsqr(x_sym[9:12]))

model.cost_expr_ext_cost   = stage_cost
model.cost_expr_ext_cost_e = ca.SX(0)   # terminaalikustannus = 0

# --- 5b. Pallo-este: pehmeä rajoitin slackin kautta (dist² - r² >= 0) ---
ball_cx = p_sym[3]
ball_cy = p_sym[4]
ball_cz = p_sym[5]
ball_r  = p_sym[6]
h_ball = (x_sym[0] - ball_cx)**2 + (x_sym[1] - ball_cy)**2 + (x_sym[2] - ball_cz)**2 - ball_r**2
model.con_h_expr   = h_ball
model.con_h_expr_e = h_ball

# --- 6. AcadosOcp ---
ocp = AcadosOcp()
ocp.model = model

ocp.dims.N = N

# Kustannustyyppi
ocp.cost.cost_type   = 'EXTERNAL'
ocp.cost.cost_type_e = 'EXTERNAL'

# Rajoitukset
max_thrust  = 9.0
max_vtheta  = 5.0
max_angle   = 65 * np.pi / 180

# Ohjausrajoitukset
ocp.constraints.lbu = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
ocp.constraints.ubu = np.array([max_thrust]*4 + [max_vtheta])
ocp.constraints.idxbu = np.arange(5)

# Tilarajoitukset (phi=6, theta=7)
ocp.constraints.lbx = np.array([-max_angle, -max_angle])
ocp.constraints.ubx = np.array([ max_angle,  max_angle])
ocp.constraints.idxbx = np.array([6, 7])

# Alkutila (päivitetään joka iteraatio)
x0 = np.zeros(13)
x0[2] = 1.0   # korkeus 1 m
ocp.constraints.x0 = x0

# Pallo-rajoittimen rajat: h >= 0 (droni pysyy pallon ulkopuolella)
ocp.constraints.lh   = np.array([0.0])
ocp.constraints.uh   = np.array([1e9])
ocp.constraints.lh_e = np.array([0.0])
ocp.constraints.uh_e = np.array([1e9])

# Pehmeä rajoitin: slack s aktivoituu kun h < 0 (pallo rikotaan)
# Kustannus: W_ball * s (lineaarinen) + W_ball_quad * s² (kvadraattinen)
W_ball      = 0.0      # lineaarinen paino
W_ball_quad = 5000.0   # kvadraattinen paino
ocp.constraints.idxsh   = np.array([0])  # h-rajoitin 0 on pehmeä
ocp.constraints.idxsh_e = np.array([0])
# [zl, zu]: lineaarinen kustannus slack alaspäin / ylöspäin
ocp.cost.zl   = np.array([W_ball])
ocp.cost.zu   = np.array([0.0])     # yläraja ei aktivoidu
ocp.cost.Zl   = np.array([W_ball_quad])
ocp.cost.Zu   = np.array([0.0])
ocp.cost.zl_e = np.array([W_ball])
ocp.cost.zu_e = np.array([0.0])
ocp.cost.Zl_e = np.array([W_ball_quad])
ocp.cost.Zu_e = np.array([0.0])

# Online-parametrit
ocp.parameter_values = np.array([0.0, 30.0, 5.0, 0.0, 0.0, 5.0, 1.0])   # [yaw_world, mu, Cl, bx, by, bz, br]

# Aika-askel
ocp.solver_options.tf = N * dt

# --- 7. Solver-asetukset ---
ocp.solver_options.qp_solver        = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx   = 'GAUSS_NEWTON'     # EXTERNAL cost vaatii tämän
ocp.solver_options.integrator_type  = 'ERK'            # Explicit Runge-Kutta (= RK4)
ocp.solver_options.nlp_solver_type  = 'SQP_RTI'        # Reaaliaikaan sopiva
ocp.solver_options.nlp_solver_max_iter = 1             # RTI: 1 iteraatio / askel
ocp.solver_options.sim_method_num_stages = 4           # RK4
ocp.solver_options.sim_method_num_steps  = 1
ocp.solver_options.print_level      = 0
ocp.solver_options.levenberg_marquardt = 10.0           # Regularisoi Hessian
ocp.solver_options.qp_solver_iter_max = 50              # Enemmän QP-iteraatioita
ocp.solver_options.qp_tol            = 1e-4

# --- 8. Generoi & käännä C-koodi ---
ocp_solver = AcadosOcpSolver(ocp, json_file='nmpc_acados.json')
print("Acados solver generoitu ja käännetty.")

# --- 9. Esimerkki MPC-silmukasta ---
def run_mpc_step(ocp_solver, x_current, yaw_world_val, mu_val=30.0, cl_val=5.0):
    # Aseta alkutila
    ocp_solver.constraints_set(0, 'lbx', x_current)
    ocp_solver.constraints_set(0, 'ubx', x_current)

    # Aseta online-parametrit kaikille vaiheille
    for k in range(N + 1):
        ocp_solver.set(k, 'p', np.array([yaw_world_val, mu_val, cl_val]))

    # Ratkaise
    status = ocp_solver.solve()
    if status != 0:
        print(f"Acados palautti statuksen {status}")

    # Hae ensimmäinen ohjaus
    u_opt = ocp_solver.get(0, 'u')
    return u_opt