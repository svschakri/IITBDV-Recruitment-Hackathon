import numpy as np

integral = 0
previous_error = 0
last_index = 0

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def steering(path, state):
    global last_index

    sx, sy = state["x"], state["y"]

    # Wider search window so we don't lose track at high speed
    search_range = 30
    start = max(0, last_index - 2)   # small backtrack for robustness
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

    # Scale lookahead with current speed — faster = look further ahead
    speed = max(state["vx"], 1.0)
    lookahead_steps = int(np.clip(speed * 0.4, 8, 25))
    lookahead = min(best + lookahead_steps, len(path) - 1)
    req = path[lookahead]

    heading = np.arctan2(req["y"] - sy, req["x"] - sx)
    error = normalize_angle(heading - state["yaw"])

    # Gentler gain → less oscillation → less fake-curvature → higher speed
    steer = 1.2 * error
    return np.clip(steer, -0.5, 0.5)


def throttle_algorithm(target_speed, current_speed, dt):
    global integral, previous_error

    # More aggressive gains — don't dawdle on straights
    Kp = 2.0
    Ki = 0.05
    Kd = 0.15

    error = target_speed - current_speed
    integral += error * dt
    integral = np.clip(integral, -20, 20)   # anti-windup
    derivative = (error - previous_error) / dt
    previous_error = error

    output = Kp * error + Ki * integral + Kd * derivative

    if output > 0:
        return np.clip(output, 0.0, 1.0), 0.0
    else:
        return 0.0, np.clip(-output, 0.0, 1.0)


def control(path, state, cmd_feedback, step):
    global integral, previous_error, last_index

    steer = steering(path, state)
    curvature = abs(steer)

    # Much higher ceiling — 100 km/h on straights, 25 minimum in hairpins
    target_speed = 120 * np.exp(-3.5 * curvature)
    target_speed = np.clip(target_speed, 25, 100)

    throttle, brake = throttle_algorithm(target_speed, state["vx"], 0.05)
    return throttle, steer, brake
