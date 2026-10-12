import multiprocessing as mp
import time
import sys
import os
import csv
import json
import random
import mujoco


def load_config(config_file="config.json"):    
    with open(config_file, "r") as f:
        return json.load(f)

def load_xml(xml_file="cart_pole.xml"):
    with open(xml_file, "r") as f:
        return f.read()

def physics_process(shared_state, control_force, running_flag, render_flag, control_hz, config_data, use_rt, use_perturbation):
    """
    Process responsible of executing the physical simulation (MuJoCo).    
    """
    viewer_hz = 60.0

    xml_model = load_xml()
    model = mujoco.MjModel.from_xml_string(xml_model)
    data = mujoco.MjData(model)
    
    # Initialize the pendulum with a slight deviation so the controller has to act
    data.qpos[1] = 0.05
    
    start_time = time.perf_counter()
    last_perturbation_time = 0.0
    
    perturbation_config = config_data.get("perturbation", {})
    perturb_interval = perturbation_config.get("interval_s", 5.0)
    perturb_min = perturbation_config.get("min_rads", -1.0)
    perturb_max = perturbation_config.get("max_rads", 1.0)
    
    viewer = None
    last_render_time = 0
    if render_flag.value:
        import mujoco.viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)

        # Configure the camera to track the cart/pole automatically
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cart")
        viewer.cam.distance = 10.0
        viewer.cam.elevation = -15
        
    while running_flag.value:
        t_now = time.perf_counter() - start_time
        
        # Advance the simulation until it reaches real time
        while data.time < t_now:
            # Read the control force calculated by the other process and inject it
            data.ctrl[0] = control_force.value
            
            # Apply perturbation to the pendulum if activated
            if use_perturbation and (data.time - last_perturbation_time) >= perturb_interval:
                perturbation_val = random.uniform(perturb_min, perturb_max)
                data.qvel[1] += perturbation_val
                print(f"[Physics] Perturbation of {perturbation_val:.2f} rad/s applied at t={data.time:.1f}s")
                last_perturbation_time = data.time
            
            # Automatic reset of the pendulum if it falls, for testing and trials.
            # If the angle is greater than ~1.2 radians (approx 70 degrees), we consider it fallen
            if abs(data.qpos[1]) > 1.2:
                data.qpos[0] = 0.0   # Return cart to center
                data.qvel[0] = 0.0
                data.qpos[1] = 0.05  # Restart upward with a slight deviation
                data.qvel[1] = 0.0
                mujoco.mj_forward(model, data)
                print(f"[Physics] Pendulum fell at t={data.time:.1f}s. Resetting position.")                       
                
            mujoco.mj_step(model, data)
            
        # Update the viewer at 60 Hz so it doesn't block the interface
        if viewer is not None and (t_now - last_render_time) > (1.0 / viewer_hz):
            if control_hz is not None:
                text1 = "Control Freq"
                text2 = f"{control_hz.value:.1f} Hz"                               
                    
                viewer.set_texts([
                    (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, text1, text2)
                ])
                
            viewer.sync()
            last_render_time = t_now
            
        # Dump the pendulum state to shared memory
        shared_state[0] = data.qpos[0]  # Cart position (x)
        shared_state[1] = data.qvel[0]  # Cart velocity (x_dot)
        shared_state[2] = data.qpos[1]  # Pendulum angle (theta)
        shared_state[3] = data.qvel[1]  # Angular velocity (theta_dot)
        
        # Brief sleep to avoid unnecessarily CPU consuming        
        time.sleep(0.0001)
        
    if viewer is not None:
        viewer.close()

def control_process(shared_state, control_force, running_flag, control_hz, config_data, target_hz, use_rt):
    """
    Thread responsible for controlling the pendulum.
    """

    period_ns = int(1_000_000_000 / target_hz)
    print(f"[Control] Control process period: {period_ns/1e6} ms.")

    duration_s = config_data["simulation_duration_s"]
    total_cycles = duration_s * target_hz
    
    # Gains are intentionally not optimized.
    Kp_cart = 20.0
    Kd_cart = 40.0
    Kp_pole = 800.0
    Kd_pole = 200.0            
    
    print(f"[Control] Starting control loop: {total_cycles} cycles ({duration_s}s at {target_hz}Hz).")
    
    start_time_ns = time.perf_counter_ns()
    expected_time_ns = start_time_ns + period_ns
    last_cycle_time = start_time_ns
    
    while True: 
        # Check end condition (duration_s seconds have passed)
        now_ns = time.perf_counter_ns()
        if (now_ns - start_time_ns) / 1e9 >= duration_s:
            break

        # We perform a sleep to release the CPU until the next period
        now_ns = time.perf_counter_ns()
        sleep_time_s = (expected_time_ns - now_ns) / 1e9
        
        if sleep_time_s > 0:                     
            time.sleep(sleep_time_s)
            
                        
                  
        actual_time_ns = time.perf_counter_ns()
        
        # Estimate frequency for display purposes (not used for control)
        if control_hz is not None:
            delta_ns = actual_time_ns - last_cycle_time
            if delta_ns > 0:
                current_hz = 1e9 / delta_ns
                if control_hz.value == 0.0:
                    control_hz.value = current_hz
                else:
                    # Exponential smoothing for better readability
                    control_hz.value = 0.95 * control_hz.value + 0.05 * current_hz
            last_cycle_time = actual_time_ns
                    
        
        # Read the state of the pendulum
        x = shared_state[0]             # Cart position (x)
        x_dot = shared_state[1]         # Cart velocity (x_dot)
        theta = shared_state[2]         # Pendulum angle (theta)
        theta_dot = shared_state[3]     # Angular velocity (theta_dot)
        
        # Use an LQR (Linear Quadratic Regulator)                
        force = (Kp_cart * x) + (Kd_cart * x_dot) + (Kp_pole * theta) + (Kd_pole * theta_dot)
        
        # Limit the force
        force = max(-100, min(100, force))        
        
        # Inject control
        control_force.value = force
        
        # Measure execution time (microsecond precision)
        iter_end_ns = time.perf_counter_ns()
        exec_time_us = (iter_end_ns - actual_time_ns) / 1e3               
        
        # Calculate the exact next wake-up time
        expected_time_ns += period_ns
            

    print("[Control] Loop finished. Shutting down simulation...")
    running_flag.value = 0
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Real-Time Latency Benchmark with MuJoCo")
    
    parser.add_argument("--render", action="store_true", help="Show the 3D simulation in real time")
    parser.add_argument("--hz", type=int, help="Control cycle frequency in Hz (overrides config.json)")
    parser.add_argument("--rt", action="store_true", help="Activate real-time policies (requires sudo)")
    parser.add_argument("--perturb", action="store_true", help="Activate periodic perturbation")
    args = parser.parse_args()
    
    config_data = load_config()
    target_hz = args.hz if args.hz else config_data["control_hz"]
        
    
    # Avoid potential multiprocessing warnings on different OS
    mp.set_start_method('spawn', force=True)
    
    # Shared memory variables
    # 4 positions: [x, x_dot, theta, theta_dot]
    shared_state = mp.Array('d', 4)
    # Actuator force
    control_force = mp.Value('d', 0.0)
    # Flag to shut down the physics thread
    running_flag = mp.Value('i', 1)
    # Render flag
    render_flag = mp.Value('i', 1 if args.render else 0)
    
    # Current control frequency to display it
    control_hz = mp.Value('d', 0.0)
    
    # Instantiate processes
    p_physics = mp.Process(target=physics_process, args=(shared_state, control_force, running_flag, render_flag, control_hz, config_data, args.rt, args.perturb))
    p_control = mp.Process(target=control_process, args=(shared_state, control_force, running_flag, control_hz, config_data, target_hz, args.rt))
    
    # Start processes
    p_physics.start()
    p_control.start()
    
    # Wait for completion
    p_control.join()
    p_physics.join()
