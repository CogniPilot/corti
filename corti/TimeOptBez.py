import numpy as np
import sympy
import sympy as sy
from sympy import symbols, factorial, Function, summation, binomial, product
import scipy.optimize
import matplotlib.pyplot as plt
n, i, j, m = symbols('n, i, j, m', integer=True, real=True)

t = symbols('t')
P = Function('P') #Position Function as a function of time

deg = 6 #Bezier degree

B = summation(binomial(n, i)*(1 - t)**(n-i)*t**i*P(i), (i, 0, n))
P_vect = (sympy.Matrix([P(i) for i in range(deg)])).T #Bezier 6 uses range(6)?

def difference_operator(P_vect, j=1): #check if j must be 1
    return sympy.Matrix([P_vect[i+1] - P_vect[i] for i in range(len(P_vect) - j)]).T

class Bezier_sym:
#https://en.wikipedia.org/wiki/B%C3%A9zier_curve

    def __init__(self, P, T: float):
        self.P = P
        self.m = P.shape[0]
        self.n = P.shape[1]-1
        self.T = T
    
    def eval(self, t):
        #https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
        beta = t/self.T
        A = sympy.Matrix(self.P)
        for j in range(1, self.n + 1):
            for k in range(self.n + 1 - j):
                A[:, k] = A[:, k] * (1 - beta) + A[:, k + 1] * beta
        return A[:, 0]

    def deriv(self, m=1):
        D_sy = sympy.Matrix(self.P)
        diff_calc = sympy.zeros(D_sy.shape[0]-1,1).T
        for j in range(1, m+1): #should range start from 1 or 0???
            diff_calc = diff_calc.row_join(difference_operator(D_sy,j))
            D_soln = (self.n +1 -j)*diff_calc  #constant term is 5? or 4? is it P_vect.shape[1]-1?
        return Bezier_sym(D_soln/self.T**m, self.T)
    

def find_Q_Bez(bez_deg, n_legs=1): #Bezier 6 = bez_deg =6
    """
    Finds the cost matrix Q
    @param deriv: for cost J, 0=position, 1=velocity, etc.
    @param poly_deg: degree of polynomial
    @n_legs: number of legs in trajectory (num. waypoints - 1)
    @return Q matrix for cost J = p^T Q p
    """
    k, l, m, n, n_c, n_l = sympy.symbols('k, l, m, n, n_c, n_l', integer=True)
    # k summation dummy variable
    # n deg of polynomial

    beta = sympy.symbols('beta')                # scaled time on leg, 0-1
    c = sympy.MatrixSymbol('c', n_c, 1)         # coefficient matrices, length is n+1, must be variable (n_c)
    T = sympy.symbols('T')                      # time of leg
    t = sympy.symbols('t')                      # evaluate at time t 
    P= Function('P')
    P_c = (sympy.Matrix([P(i) for i in range(bez_deg)])).T #Bezier Positon Matrices length = n     #must be variable (n_c)
    Bez6 = Bezier_sym(P_c,T)
    B_d3 = Bez6.deriv().deriv().deriv()         # 3rd derivative for jerk; 4th deriv for snap
    
    Jerk = B_d3.eval(t)                         #evaluate Bezier at t
    J = (Jerk@Jerk.T).doit()                    #square of jerk term
    J = sympy.integrate(J, (t, 0, 1)).doit()    # cost
    
    p = sympy.Matrix([c[i, 0] for i in range(bez_deg+1)])  # vector of terms
    Q = sympy.Matrix([J]).jacobian(P_c).jacobian(P_c)/2  # find Q using second derivative, P_c or p for jacobian
    # assert (p.T@Q@p)[0, 0].expand() == J  # assert hessian matches cost

    Ti = sympy.MatrixSymbol('T', n_l, 1)
    return J, Q

def derive_bezier6_sym():
    n = 6 #Bezier Degree
    T = sympy.Symbol('T') #ca.SX.sym('T')
    t = sympy.Symbol('t') #ca.SX.sym('t')                       
    P= Function('P')       # P = ca.SX.sym('P', 1, n)
    P_c = (sympy.Matrix([P(i) for i in range(n)])).T #Bezier Positon Matrices length = n
    B = Bezier_sym(P_c, T)

    # derivatives
    B_d = B.deriv()
    B_d2 = B_d.deriv()
    B_d3 = B_d2.deriv()
    B_d4 = B_d3.deriv()

    # boundary conditions

    # trajectory
    p = B.eval(t)
    v = B_d.eval(t)
    a = B_d2.eval(t)
    r = sy.Matrix.vstack(p,v,a)    #ca.vertcat(p, v, a)

    # given position/velocity boundary conditions, solve for bezier points
    wp_0 = sympy.MatrixSymbol('ptin', 2, 1) #ca.SX.sym('p0', 2, 1)  # pos/vel at waypoint 0
    wp_1 = sympy.MatrixSymbol('ptone', 2, 1) #ca.SX.sym('p1', 2, 1)  # pos/vel at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B_d2.eval(0), 0)]  # zero accel @ wp0
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1
    constraints += [(B_d2.eval(T), 0)]  # zero accel @ wp1

    assert len(constraints) == 6

    Y = sy.Matrix.vstack(*[c[0] for c in constraints])  #ca.vertcat(*[c[0] for c in constraints])
    b = sy.Matrix([c[1] for c in constraints])   #ca.vertcat(*[c[1] for c in constraints])
    A = Y.jacobian(P_c)
    A_inv = A.inv()
    P_sol = (A_inv@b).T
    return {
        'bezier6_solve': P_sol,#.subs(wp_0,[1,2]).subs(wp_1,[1,2]) #ca.Function('bezier6_solve', [wp_0, wp_1, T], [P_sol], ['wp_0', 'wp_1', 'T'], ['P']),
        'bezier6_traj': r, #ca.Function('bezier6_traj', [t, T, P], [r], ['t', 'T', 'P'], ['r']),
        # 'control_points': constraints
    }

def find_cost_function(bez_deg, wpVal0, wpVal1, n_legs = 1):
    ''' 
    Takes bezier degree, initial waypoint, and final waypoint
    waypoints are [position 1, velocity 1] in sympy Matrix type
    outputs cost function, J_total
    '''
    T = sympy.Symbol('T')
    # Ti = sympy.MatrixSymbol('T', n_legs, 1)

    wp_0 = sympy.MatrixSymbol('ptin', 2, 1) #pos/vel at waypoint 0
    wp_1 = sympy.MatrixSymbol('ptone',2,1)    

    Q_Bez = find_Q_Bez(6,n_legs)[1]
    p_constr = derive_bezier6_sym()['bezier6_solve'] #Bezier Coefficient
    for i in range(len(wpVal0)):
        p_constr = p_constr.subs(wp_0[i],wpVal0[i]).subs(wp_1[i],wpVal1[i])
    pBez =p_constr.subs(T,T)

    # display(sympy.Matrix(Ti))
    #T could be fixed to vary with n_legs
    J_total = (((pBez@Q_Bez@pBez.T)[0,0]).simplify()) #Jtotal term for optimized cost calculation

    return J_total

def find_opt_time(bez_deg, bc, k_time, n_legs = 1):
    '''
    Takes bezier degree, initial waypoint, and final waypoint
    waypoints are [[position], [velocity]]
    
    Using Scipy.Optimize function, outputs optimized leg time
    '''

    T = sympy.Symbol('T')
    Ti = sympy.symbols('T_0:{:d}'.format(n_legs))
    k = sympy.symbols('k') #weight on time

    J_total_x = find_cost_function(bez_deg, bc[:,0, 0], bc[:, 1, 0], n_legs)
    J_total_y = find_cost_function(bez_deg, bc[:,0, 1], bc[:, 1, 1], n_legs)
    # J_total_z = find_cost_function(bez_deg, bc[:,0, 2], bc[:, 1, 2], n_legs)

    J_total = J_total_x + J_total_y + k * sum(Ti) # Sum of all components' jerk term
    J_total = J_total.subs(T,sy.Matrix(Ti)).doit()
    f_J = sympy.lambdify([Ti,k],J_total)

    # k_time = 10 #weight on time
    
    sol = scipy.optimize.minimize(lambda T: f_J(T,k_time), [1]*n_legs, bounds=[[0.1, 100]]* n_legs)
    T_opt = sol['x']
    
    return T_opt ## Optimzied T leg time for eval in Bezier



