import numpy as np

def parametric_circle(th,c,r):
    xc, yc = c
    x = xc + r*np.cos(th)
    y = yc + r*np.sin(th)
    return x, y

def inv_parametric_circle(p,c,r):
    x, y = p
    xc, yc = c
    return np.arctan2((y - yc)/r, (x-xc)/r)

def gen_arc(numpoints, c, r, start, end):
    start_th = inv_parametric_circle(start, c, r)
    end_th   = inv_parametric_circle(end, c, r)

    if end_th - start_th > np.pi:
        end_th -= 2*np.pi

    elif end_th - start_th < -np.pi:
        end_th += 2*np.pi

    arc_T = np.linspace(start_th, end_th, numpoints)
    X, Y = parametric_circle(arc_T, c, r)

    return X, Y

def gen_curve_points(p1, p2, p3, r, numpoints):
    a = p2-p1 
    b = p2-p3 
    a_norm = np.sqrt(np.sum(a**2))
    b_norm = np.sqrt(np.sum(b**2))
    theta = np.arccos(np.dot(a,b)/(a_norm*b_norm))

    d = r/np.tan(theta/2) # distance away from turn to start
    h = r/np.sin(theta/2) # distance from intersection to centerpoint of rotation
    
    sign = np.sign(np.cross(a,b)) # sign of rotation direction, clockwise if negative
    rot = np.array([[np.cos(sign*theta/2), -np.sin(sign*theta/2)],
                    [np.sin(sign*theta/2),  np.cos(sign*theta/2)]])
    
    d_a = d*(a/a_norm)
    d_b = d*(b/b_norm)

    start = p2-d_a
    end = p2-d_b

    c = p2 - h*rot@(a/a_norm)
    
    X, Y = gen_arc(numpoints, c, r, start, end)

    return start, end, c, X, Y

def gen_waypoints(control_points, r, num_turn_points):
    waypoints = [control_points[0]]
    for i in range(1, len(control_points)-1):
        p1, p2, p3 = np.array(control_points[i-1]), np.array(control_points[i]), np.array(control_points[i+1])

        start, end, c, X, Y = gen_curve_points(p1,p2,p3, r, numpoints=num_turn_points)
        for x, y in zip(X, Y):
            waypoints.append((x,y))

    waypoints.append(control_points[-1])

    return waypoints

def gen_linear_leg(p1, p2, v, dt, t_start, last_theta):
    p1, p2 = np.array(p1), np.array(p2)
    theta = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
    
    if last_theta:
        while theta < last_theta-np.pi: theta += 2*np.pi
        while theta > last_theta+np.pi: theta -= 2*np.pi

    distance = np.sqrt(np.sum((p1-p2)**2))
    time = distance/v
    numpoints = max(2,int(time/dt))

    xs = np.linspace(p1[0], p2[0], numpoints)
    ys = np.linspace(p1[1], p2[1], numpoints)
    thetas = np.ones(xs.shape)*theta
    ts = np.linspace(t_start, t_start+(numpoints-1)*dt, numpoints)
    
    leg = np.vstack([xs, ys, thetas, ts]).T
    return leg

def gen_reference_trajectory(waypoints, v, dt):
    t_start = 0
    trajectory = None

    for i in range(len(waypoints)-1):
        if i == 0:
            leg = gen_linear_leg(waypoints[i], waypoints[i+1], v, dt, t_start, last_theta=None)
            trajectory = leg
        else:
            leg = gen_linear_leg(waypoints[i], waypoints[i+1], v, dt, t_start, last_theta=trajectory[-1,2])
            trajectory = np.vstack([trajectory, leg[1:]])
        
        t_start = trajectory[-1,-1]
    
    return trajectory