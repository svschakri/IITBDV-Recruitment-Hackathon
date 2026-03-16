# PPC Hackathon Submission

## Student Details

**Name:** Somisetty Venkata Sai Chakri\
**Roll No:** 25B1070

------------------------------------------------------------------------

# Perception

In this task, I estimate the distance to cones detected by the YOLO
model.

## Assumptions

-   The cone height is fixed.
-   Cones are upright.
-   Cones are not stacked.
-   The camera focal length is fixed.

## Method

First, extract the bounding box of each detected cone:

``` python
x1, y1, x2, y2 = map(int, box.xyxy[0])
```

Then compute the height of the cone in pixels:

``` python
h = abs(y1 - y2)
```

Using the pinhole camera model, the distance of the cone is estimated
as:

d = (H \* f) / h

Where: - **H** = real-world height of the cone\
- **f** = focal length of the camera\
- **h** = height of the cone in pixels

``` python
d = (H * f) / h
```

## Visualization

Bounding boxes and labels are drawn on detected cones:

``` python
cv2.rectangle(image, (x1,y1), (x2,y2), (0,255,0), 2)

cv2.putText(
    image,
    label,
    (x1,y1-10),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.5,
    (0,255,255),
    2
)
```

## Terminal Output

Distances of detected cones are printed in the terminal:

``` python
for i in range(len(detections)):
    print(f"Cone {i+1}: distance = {detections[i]:.2f}")
```

------------------------------------------------------------------------

# PPC

## Planner

The path is generated using **midpoints between cones**.

Steps: 1. For each blue cone, the closest yellow cone is identified. 2.
The midpoint between them is calculated. 3. Between each pair of
midpoints, **three additional interpolated points** are inserted to
smooth the path.

This produces a smoother trajectory for the vehicle.

------------------------------------------------------------------------

# Controller

## Throttle Control

A **PID controller** is used for speed regulation.

### PID Parameters

-   **Kp = 1.2** → Proportional gain (present error)\
-   **Ki = 0.02** → Integral gain (accumulated past error)\
-   **Kd = 0.1** → Derivative gain (future trend of error)

### Implementation

``` python
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
    throttle = np.clip(output, 0.0, 1.0)
    brake = 0
else:
    brake = np.clip(-output, 0.0, 1.0)
    throttle = 0

return throttle, brake
```

------------------------------------------------------------------------

## Steering Control

Steering follows a **look-ahead point strategy**.

Steps: 1. Find the closest point on the path. 2. Select a look-ahead
point several steps ahead. 3. Compute the heading toward that point.

### Implementation

``` python
lookahead = (best + 8) % len(path)
req = path[lookahead]

heading = np.arctan2(req["y"] - sy, req["x"] - sx)
error = normalize_angle(heading - state["yaw"])

steer = 2 * error
return np.clip(steer, -0.5, 0.5)
```

------------------------------------------------------------------------

## Target Speed

Target speed depends on **path curvature**.

Assumption: - Curvature is proportional to the absolute steering angle.

Higher curvature → lower speed.

### Implementation

``` python
steer = steering(path, state)

curvature = abs(steer)

target_speed = 100 * np.exp(-2 * curvature)
target_speed = np.clip(target_speed, 15, 50)
```

This ensures: - Higher speed on straight sections - Lower speed while
turning

------------------------------------------------------------------------

# Summary

-   **Perception:** YOLO-based cone detection with distance estimation
    using the pinhole camera model.
-   **Planning:** Path generated using midpoints between cone pairs with
    interpolation.
-   **Control:** PID throttle control, look-ahead steering, and
    curvature-based speed control.
