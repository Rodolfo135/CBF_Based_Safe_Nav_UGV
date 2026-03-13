import keyboard
import time
import math
import numpy as np
from scipy.optimize import minimize
from cyberwave import Cyberwave
from state_estimator import StateEstimator

# --- 1. SETUP ---
cw = Cyberwave(api_key="747fa7a3-cb9f-415a-8183-a367067f1f69")
ugv_beast = cw.twin(
    "waveshare/ugv-beast", 
    twin_id="e92fd3e2-824e-40bd-9515-9ebf42634dea",
    environment_id="a99d36b0-85ad-425b-bf16-5033f79aa0cd"
)

estimator = StateEstimator(ugv_beast)

def cbf_qp_filter(cur_x, cur_y, cur_yaw, v_nom, w_nom_rad, obs_x=5.0, obs_y=0.0, dt=0.5):
    """
    Filters inputs using a Discrete-Time Soft QP (DCBF) and prints the solver status
    at every time step.
    """
    D_s = 1.0     
    gamma = 1.0   
    l = 0.3       
    
    theta = math.radians(cur_yaw)

    # 1. Evaluate Current Barrier State
    xp_curr = cur_x + l * math.cos(theta)
    yp_curr = cur_y + l * math.sin(theta)
    h_curr = (xp_curr - obs_x)**2 + (yp_curr - obs_y)**2 - D_s**2

    # 2. Objective Function
    def objective(u):
        return 0.5 * ((u[0] - v_nom)**2 + (u[1] - w_nom_rad)**2) + 1000.0 * (u[2]**2)

    # 3. Discrete-Time Non-Linear Constraint
    def cbf_constraint(u):
        v, w_rad, slack = u
        
        next_yaw = theta + (w_rad * dt)
        next_x = cur_x + (v * math.cos(theta) * dt)
        next_y = cur_y + (v * math.sin(theta) * dt)
        
        next_xp = next_x + l * math.cos(next_yaw)
        next_yp = next_y + l * math.sin(next_yaw)
        
        h_next = (next_xp - obs_x)**2 + (next_yp - obs_y)**2 - D_s**2
        
        return h_next - (1.0 - gamma) * h_curr + slack

    u0 = np.array([v_nom, w_nom_rad, 0.0])
    bounds = ((0.0, None), (None, None), (0.0, None)) 
    constraints = {'type': 'ineq', 'fun': cbf_constraint}

    # Solve the QP
    res = minimize(objective, u0, method='SLSQP', bounds=bounds, constraints=constraints)

    # --- DEBUG CONSOLE OUTPUT ---
    print("\n" + "="*40)
    print("QP OPTIMIZER LOG:")
    print(f"State     -> X: {cur_x:.2f}, Y: {cur_y:.2f}, Yaw: {cur_yaw:.1f}°")
    print(f"Nominal   -> v: {v_nom:.3f}, w_rad: {w_nom_rad:.3f}")
    print(f"Barrier   -> h_curr: {h_curr:.3f} (Must stay >= 0)")
    
    if res.success:
        print("Status    -> [SUCCESS]")
        print(f"Safe Cmds -> v: {res.x[0]:.3f}, w_rad: {res.x[1]:.3f}")
        print(f"Slack Var -> {res.x[2]:.6f}")
        
        # Warning if the slack variable is actively being used to prevent a crash
        if res.x[2] > 0.001:
            print("WARNING   -> Slack variable active! Boundary breached.")
            
        print("="*40)
        return res.x[0], res.x[1] 
    else:
        print("Status    -> [INFEASIBLE / FAILED]")
        print(f"Reason    -> {res.message}")
        print(f"Failed At -> v: {res.x[0]:.3f}, w_rad: {res.x[1]:.3f}, slack: {res.x[2]:.6f}")
        print("Action    -> Executing safety stop (v=0).")
        print("="*40)
        return 0.0, w_nom_rad


def run_autonomous_loop(target_x, target_y):
    print(f"\n>>> Autonomous Navigation Active: Goal [{target_x}, {target_y}] <<<")
    
    dt = 0.5 # Corresponds to your REST API Rate Limit
    obs_x, obs_y = 5.0, 0.0
    
    while True:
        cur_x, cur_y, cur_yaw = estimator.update()
        
        err_x, err_y = target_x - cur_x, target_y - cur_y
        dist_to_goal = math.sqrt(err_x**2 + err_y**2)
        
        if dist_to_goal < 0.2:
            print("Target Reached!")
            break

        target_yaw = math.degrees(math.atan2(err_y, err_x))
        yaw_err = (target_yaw - cur_yaw + 180) % 360 - 180
        
        v_nom = min(0.8 * dist_to_goal, 1.0)
        w_nom_deg = max(min(0.5 * yaw_err, 30.0), -30.0)

        # --- SYMMETRY BREAKING ---
        # Break the unicycle deadlock if the obstacle is dead-ahead
        dist_to_obs = math.sqrt((obs_x - cur_x)**2 + (obs_y - cur_y)**2)
        obs_yaw = math.degrees(math.atan2(obs_y - cur_y, obs_x - cur_x))
        obs_yaw_err = (obs_yaw - cur_yaw + 180) % 360 - 180

        # If it's within 3m and directly in our path (< 10 degrees)
        if dist_to_obs < 3.0 and abs(obs_yaw_err) < 10.0:
            w_nom_deg += 25.0  # Nudge left to expose the flank to the CBF
            
        w_nom_rad = math.radians(w_nom_deg)

        # --- CBF SAFETY FILTER ---
        v_safe, w_safe_rad = cbf_qp_filter(cur_x, cur_y, cur_yaw, v_nom, w_nom_rad, obs_x, obs_y, dt)
        w_safe_deg = math.degrees(w_safe_rad)

        # D. ACTUATION
        new_yaw = cur_yaw + (w_safe_deg * dt)
        new_x = cur_x + (v_safe * math.cos(math.radians(cur_yaw)) * dt)
        new_y = cur_y + (v_safe * math.sin(math.radians(cur_yaw)) * dt)

        ugv_beast.rotate(yaw=new_yaw)
        ugv_beast.move([new_x, new_y, 0])

        if keyboard.is_pressed('q'): 
            print("Aborting...")
            break
        
        time.sleep(dt)

# --- MAIN INTERACTION ---
print("Ready. Press 'P' for Autonomous Path, 'Q' to quit.")
while True:
    if keyboard.is_pressed('p'):
        run_autonomous_loop(10.0, 0.0)
    if keyboard.is_pressed('q'):
        break
    time.sleep(0.05)