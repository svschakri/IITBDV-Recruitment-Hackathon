## Student Details

**Name** : Somisetty Venkata Sai Chakri

**Roll no** : 25b1070

## Perception :
here I am assuming cone height is fixed and it is upright and there is no cone stacked and focal length of camera is fixed 
    
    x1, y1, x2, y2 = map(int , box.xyxy[0])
    h=abs(y1-y2)
    
    #I used this thing for finding height in pixels 
    #and I found the distance using the formulae given below
    
    
    d=(H*f)/h

    #and I labeled using this 

    
    cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
    cv2.putText(image,label,(x1,y1-10),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),2)

    # for printing in terminal i used this
    
    for i in range (len(detections)) :
    print(f"Cone {i+1}: distance = {detections[i]:.2f}")

## PPC :
 ### Planner:
  the path is mid point of blue cones and respectively closet yellow cone and between each two points I insertered three points
 ### Controller:
  #### Thorttle Algo:
   I use PID controlling for this Question and I used kp=1.2 , ki=0.02 and kd=0.1 in this case kp is constant for error(present error) ,ki is constant for integral error(past), kd is constant for differential error(furture)
   
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

  #### steering angle: 
  first I will find the closet point and i will point towards the 8 points after that in terms of modulo scale 
  lookahead = (best + 8)%len(path)
    req = path[lookahead]

    heading = np.arctan2(req["y"]-sy, req["x"]-sx)
    error = normalize_angle(heading - state["yaw"])

    steer = 2 * error
    return np.clip(steer, -0.5, 0 .5)
  ### target speed :
  my target speed depends on curvature and I assumed curvature is directly proportional to abs(steering)
    
    
    steer = steering(path, state)
    curvature = abs(steer)
    target_speed = 100 * np.exp(-2 * curvature)
    target_speed = np.clip(target_speed, 15, 50 )
