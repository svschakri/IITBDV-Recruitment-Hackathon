
'''
PPC Hackathon — Participant Boilerplate
You must implement two functions: plan() and control()
'''

# ─── TYPES (for reference) ────────────────────────────────────────────────────

# Path: list of waypoints [{"x": float, "y": float}, ...]
# State: {"x", "y", "yaw", "vx", "vy", "yaw_rate"} 
# CmdFeedback: {"throttle", "steer"}         

# ─── CONTROLLER ───────────────────────────────────────────────────────────────
import numpy as np

def Distance(a,b):
    dx = a[0]-b[0]
    dy = a[1]-b[1]
    return dx*dx + dy*dy

integral=0
previous_error=0
 
# def steering(path: list[dict], state: dict):

#     length_of_car = 2.6
#     # Calculate steering angle based on path and vehicle state
#     mi=9000
#     req="not Known"
#     for a in path :
#         if Distance([a["x"],a["y"]],[state["x"],state["y"]]) < mi :
#             mi=Distance([a["x"],a["y"]],[state["x"],state["y"]])
#             req=a
    
#     c = path.index(req)
#     req = path[(c + 6) % len(path)]

#     heading = np.arctan2(req["y"]-state["y"], req["x"]-state["x"])
#     error = normalize_angle(heading - state["yaw"])
#     steer = 2 * error
#     # Default steer value

#     # 0.5 in the max steering angle in radians (about 28.6 degrees)
#     return np.clip(steer, -0.5, 0.5)

last_index = 0

def steering(path, state):
    global last_index

    sx = state["x"]
    sy = state["y"]

    search_range = 15
    start = last_index
    end = min(len(path), start + search_range)

    mi = 1e9
    best = start

    for i in range(start, end):
        a = path[i]
        dx = a["x"] - sx
        dy = a["y"] - sy
        d = dx*dx + dy*dy

        if d < mi:
            mi = d
            best = i

    last_index = best

    lookahead = (best + 8)%len(path)
    req = path[lookahead]

    heading = np.arctan2(req["y"]-sy, req["x"]-sx)
    error = normalize_angle(heading - state["yaw"])

    steer = 2 * error
    return np.clip(steer, -0.5, 0.5)


def throttle_algorithm(target_speed, current_speed, dt):

    global integral, previous_error
    Kp=1.2
    Ki=0.02
    Kd=0.1
    
    error = target_speed - current_speed

    # Integral
    integral += error * dt

    # Derivative
    derivative = (error - previous_error) / dt

    previous_error = error

    # PID output
    output = Kp*error + Ki*integral + Kd*derivative

    throttle = 0
    brake = 0

    if output > 0:
        throttle = np.clip(output, 0.0, 3.0)
        brake = 0
    else:
        brake = np.clip(-output, 0.0, 1.0)
        throttle = 0

    return throttle, brake


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def control(
    path: list[dict],
    state: dict,
    cmd_feedback: dict,
    step: int,
) -> tuple[float, float, float]:
    """
    Generate throttle, steer, brake for the current timestep.
    Called every 50ms during simulation.

    Args:
        path:         Your planned path (waypoints)
        state:        Noisy vehicle state observation
                        x, y        : position (m)
                        yaw         : heading (rad)
                        vx, vy      : velocity in body frame (m/s)
                        yaw_rate    : (rad/s)
        cmd_feedback: Last applied command with noise
                        throttle, steer, brake
        step:         Current simulation timestep index

    Returns:
        throttle  : float in [0.0, 1.0]   — 0=none, 1=full
        steer     : float in [-0.5, 0.5]  — rad, neg=left
        brake     : float in [0.0, 1.0]   — 0=none, 1=full
    
    Note: throttle and brake cannot both be > 0 simultaneously.
    """
    throttle = 0.0
    steer    = 0.0
    brake = 0.0
   
    # TODO: implement your controller here
    steer = steering(path, state)
    curvature = abs(steer)

    target_speed = 100 * np.exp(-2 * curvature)
    target_speed = np.clip(target_speed, 15, 50 )
    
    throttle, brake = throttle_algorithm(target_speed, state["vx"], 0.05)

    return throttle, steer, brake
