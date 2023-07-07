import math
import numpy as np

def matrix_exp(A, n=30):
    s = np.zeros((3, 3))
    A_i = np.eye(3)
    for i in range(n):
        s = s + A_i/math.factorial(i)
        A_i = A_i@A
    return s


def check_shape(a, shape):
    if np.shape(a) != shape:
        raise IOError(str(np.shape(a)) + '!=' + str(shape))


def wrap(x):
    return np.where(np.abs(x) >= np.pi, (x + np.pi) % (2 * np.pi) - np.pi, x)


class LieGroup:
    
    def __repr__(self):
        return repr(self.matrix)

    def __mul__(self, other):
        return NotImplementedError('')

    
class LieAlgebra:
    
    def __repr__(self):
        return repr(self.wedge)

    def __mul__(self, other):
        return NotImplementedError('')


class Vector:
    
    def __repr__(self):
        return repr(self.matrix)

    def __mul__(self, other):
        return NotImplementedError('')


class R2(Vector):
    
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
    
    @property
    def matrix(self):
        return np.array([[self.x], [self.y]])

    def __neg__(self):
        return R2(x=-self.x, y=-self.y)
    
    def __add__(self, other):
        return R2(x=self.x + other.x, y=self.y + other.y)

    @classmethod
    def from_vector(cls, a):
        a = a.reshape(-1)
        return cls(x=a[0], y=a[1])

    
class so2(LieAlgebra):
    
    def __init__(self, theta):
        self.theta = np.reshape(wrap(theta), ())
    
    @property
    def wedge(self):
        return np.array([
            [0, -self.theta],
            [self.theta, 0]
        ])
    
    @property
    def vee(self):
        return np.array([self.theta])

    @property
    def exp(self):
        return SO2(theta=self.theta)
    

class SO2(LieGroup):
    
    def __init__(self, theta):
        self.theta = np.reshape(wrap(theta), ())
    
    @classmethod
    def one(cls):
        return cls(theta=0)

    @property
    def inv(self):
        return SO2(theta=-self.theta)

    @property
    def matrix(self):
        return np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]
        ])
    
    @property
    def params(self):
        return np.array([self.theta])

    @property
    def log(self):
        return so2(self.theta)
    
    @classmethod
    def from_matrix(cls, a):
        check_shape(a, (2, 2))
        return cls(theta=np.arctan2(a[1, 0], a[0, 0]))

    def __matmul__(self, other):
        if isinstance(other, R2):
            return R2.from_vector(self.matrix@other.matrix)
        elif isinstance(other, SO2):
            return SO2(theta=self.theta + other.theta)


class se2(LieAlgebra):
    
    def __init__(self, x: float, y: float, theta: float):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    def __neg__(self):
        return se2(-self.x, -self.y, -self.theta)

    @property
    def wedge(self):
        return np.array([
            [0, -self.theta, self.x],
            [self.theta, 0, self.y],
            [0, 0, 0]
        ])
    

    def __add__(self, other):
        return se2(x=self.x + other.x, y=self.y + other.y, theta=self.theta + other.theta)
    
    def __sub__(self, other):
        return se2(x=self.x - other.x, y=self.y - other.y, theta=self.theta - other.theta)
    
    @property
    def vee(self):
        return np.array([self.x, self.y, self.theta])
    
    @classmethod
    def from_vector(cls, a):
        a = a.reshape((3, 1))
        return cls(x=a[0], y=a[1], theta=a[2])
    
    @classmethod
    def from_matrix(cls, a):
        check_shape(a, (3, 3))
        return cls(x=a[0, 2], y=a[1, 2], theta=a[1, 0])

    @property
    def ad_matrix(self):
        x, y, theta = self.x, self.y, self.theta
        return np.array([
            [0, -theta, y],
            [theta, 0, -x],
            [0, 0, 0]
        ])

    def __matmul__(self, other):
        return se2.from_vector(self.ad_matrix@other.vee)

    @property
    def exp(self):
        theta = self.theta
        with np.errstate(divide='ignore',invalid='ignore'):
            a = np.where(np.abs(theta) < 1e-3, 1 - theta**2/6 + theta**4/120, np.sin(theta)/theta)
            b = np.where(np.abs(theta) < 1e-3, theta/2 - theta**3/24 + theta**5/720, (1 - np.cos(theta))/theta)
        V = np.array([[a, -b], [b, a]])
        p = V@np.array([self.x, self.y])
        return SE2(x=p[0], y=p[1], theta=self.theta)

    def __rmul__(self, scalar):
        s = np.reshape(scalar, ())
        return se2(x=self.x*s, y=self.y*s, theta=self.theta*s)


class SE2(LieGroup):
    
    def __init__(self, x: float, y: float, theta: float):
        self.x = float(x)
        self.y = float(y)
        self.theta = wrap(float(theta))
    
    @classmethod
    def one(cls):
        return cls(x=0, y=0, theta=0)

    @property
    def params(self):
        return np.array([self.x, self.y, self.theta])
    
    @property
    def matrix(self):
        x, y, theta = self.x, self.y, self.theta
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])

    @property
    def R(self):
        return SO2(theta=self.theta)

    @property
    def p(self):
        return R2(x=self.x, y=self.y)
    
    @property
    def inv(self):
        p = -(self.R.inv@self.p)
        return SE2(x=p.x, y=p.y, theta=-self.theta)

    def __matmul__(self, other: 'SE2'):
        p = self.R@other.p + self.p
        return SE2(x=p.x, y=p.y, theta=self.theta + other.theta)

    @classmethod
    def from_matrix(cls, a: np.array):
        check_shape(a, (3, 3))
        return SE2(theta=np.arctan2(a[1, 0], a[0, 0]),
                   x=a[0, 2], y=a[1, 2])
    
    @classmethod
    def from_vector(cls, a):
        a = a.reshape((3, 1))
        return cls(x=a[0], y=a[1], theta=a[2])

    @property
    def Ad_matrix(self):
        x, y, theta = self.x, self.y, self.theta
        return np.array([
            [np.cos(theta), -np.sin(theta), y],
            [np.sin(theta), np.cos(theta), -x],
            [0, 0, 1]
        ])
    
    def Ad(self, v: 'se2'):
        v2 = self.Ad_matrix@v.vee
        return se2(x=v2[0], y=v2[1], theta=v2[2])

    @property
    def log(self):
        x, y, theta = self.x, self.y, self.theta
        with np.errstate(divide='ignore',invalid='ignore'):
            a = np.where(np.abs(theta) < 1e-3, 1 - theta**2/6 + theta**4/120, np.sin(theta)/theta)
            b = np.where(np.abs(theta) < 1e-3, theta/2 - theta**3/24 + theta**5/720, (1 - np.cos(theta))/theta)
        V_inv = np.array([
            [a, b],
            [-b, a]
        ])/(a**2 + b**2)
        p = V_inv@np.array([x, y])
        return se2(x=p[0], y=p[1], theta=theta)  

def diff_correction(e: se2, n=100):
    # computes (1 - exp(-ad_x)/ad_x = sum k=0^infty (-1)^k/(k+1)! (ad_x)^k
    ad = e.ad_matrix
    ad_i = np.eye(3)
    s = np.zeros((3, 3))
    for k in range(n):
        s += ((-1)**k/math.factorial(k+1))*ad_i
        ad_i = ad_i @ ad
    return -np.linalg.inv(s)@((-e).exp.Ad_matrix)

def se2_diff_correction(e: se2): # U
    x = e.x
    y = e.y
    theta = e.theta
    with np.errstate(divide='ignore',invalid='ignore'):
        a = np.where(abs(theta) > 1e-3, -theta*np.sin(theta)/(2*(np.cos(theta) - 1)), 1 - theta**2/12 - theta**4/720)
        b = np.where(abs(theta) > 1e-3, -(theta*x*np.sin(theta) + (1 - np.cos(theta))*(theta*y - 2*x))/(2*theta*(1 - np.cos(theta))), -y/2 + theta*x/12 - theta**3*x/720)
        c = np.where(abs(theta) > 1e-3, -(theta*y*np.sin(theta) + (1 - np.cos(theta))*(-theta*x - 2*y))/(2*theta*(1 - np.cos(theta))), x/2 + theta*y/12 + theta**3*y/720)
    return -np.array([
        [a, theta/2, b],
        [-theta/2, a, c],
        [0, 0, 1]
    ])

def se2_diff_correction_inv(e: se2): # U_inv
    x = e.x
    y = e.y
    theta = e.theta
    with np.errstate(divide='ignore',invalid='ignore'):
        a = np.where(abs(theta) > 1e-3, np.sin(theta)/theta, 1 - theta**2/6 + theta**4/120)
        b = np.where(abs(theta) > 1e-3, -(1  - np.cos(theta))/theta, theta/2 - theta**3/24)
        c = np.where(abs(theta) > 1e-3, -(x*(theta*np.cos(theta) - theta + np.sin(theta) - np.sin(2*theta)/2) + y*(2*np.cos(theta) - np.cos(2*theta)/2 - 3/2))/(theta**2*(1 - np.cos(theta))), y/2 + theta*x/6 - theta**2*y/24 - theta**3*x/120 + theta**4*y/720)
        d = np.where(abs(theta) > 1e-3, -(x*(-2*np.cos(theta) + np.cos(2*theta)/2 + 3/2) + y*(theta*np.cos(theta) - theta + np.sin(theta) - np.sin(2*theta)/2))/(theta**2*(1 - np.cos(theta))), -x/2 + theta*y/6 + theta**2*x/24 - theta**3*y/120 - theta**4*x/720)
    return -np.array([
        [a, -b, c],
        [b, a, d],
        [0, 0, 1]
    ])