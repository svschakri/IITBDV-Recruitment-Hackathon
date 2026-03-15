
'''
PPC Hackathon — Participant Boilerplate
You must implement two functions: plan() and control()
'''

# ─── TYPES (for reference) ────────────────────────────────────────────────────

# Cone: {"x": float, "y": float, "side": "left" | "right", "index": int}
# State: {"x", "y", "yaw", "vx", "vy", "yaw_rate"}  
# CmdFeedback: {"throttle", "steer"}        

# ─── PLANNER ──────────────────────────────────────────────────────────────────
import numpy as np

def Distance(a,b) :
   return pow(pow(a[0]-b[0],2)+pow(a[1]-b[1],2),0.5)



def plan(cones: list[dict]) -> list[dict]:
    """
    Generate a path from the cone layout.
    Called ONCE before the simulation starts.

    Args:
        cones: List of cone dicts with keys x, y, side ("left"/"right"), index

    Returns:
        path: List of waypoints [{"x": float, "y": float}, ...]
              Ordered from start to finish.
    
    Tip: Try midline interpolation between matched left/right cones.
         You can also compute a curvature-optimised racing line.
    """
    path = []
    # TODO: implement your path planning here
    blue = np.array([[cone["x"], cone["y"]] for cone in cones if cone["side"] == "left"])
    yellow = np.array([[cone["x"], cone["y"]] for cone in cones if cone["side"] == "right"])

    # implement a planning algorithm to generate a path from the blue and yellow cones
    for a in blue :
        mi=1000000
        req="not Known"
        for b in yellow :
            if Distance(a,b) < mi :
                mi = Distance(a,b)
                req=b    
        X=(a[0]+req[0])/2
        Y=(a[1]+req[1])/2
        if(len(path)!=0) :
            x0=path[-1]["x"]
            y0=path[-1]["y"]
            path.append({"x":(3*x0+X)/4,"y":(3*y0+Y)/4})
            path.append({"x":(2*x0+2*X)/4,"y":(2*y0+2*Y)/4})
            path.append({"x":(1*x0+3*X)/4,"y":(1*y0+3*Y)/4})

        path.append({"x":X,"y":Y})
    
    fin=path[len(path)-1]
    sta=path[0]
    t0=fin["x"]
    u0=fin["y"]
    x1=sta["x"]
    y1=sta["y"]
    path.append({"x":(3*t0+x1)/4,"y":(3*u0+y1)/4})
    path.append({"x":(2*t0+2*x1)/4,"y":(2*u0+2*y1)/4})
    path.append({"x":(1*t0+3*x1)/4,"y":(1*u0+3*y1)/4})







    return path

