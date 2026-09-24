import numpy as np

class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)
    def __mul__(self, num):
        return Vec2(self.x * num, self.y * num)
    def __rmul__(self, num):
        return self.__mul__(num)
    def __truediv__(self, num):
        return Vec2(self.x / num, self.y / num)
    def norm2(self):
        return self.x**2 + self.y**2
    def norm(self):
        return np.sqrt(self.norm2())
    def normalize(self):
        return self / self.norm()
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    def cross(self, other):
        return self.x * other.y - self.y * other.x
    def dist_to(self, other):
        return (self - other).norm()
    def dist2_to(self, other):
        return (self - other).norm2()
    def rotate(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        x = self.x
        y = self.y
        return Vec2(c * x - s * y, s * x + c * y)
    def copy(self):
        return Vec2(self.x, self.y)
    def __repr__(self):
        return f"({self.x:.4f},{self.y:.4f})"

class Mat2:
    def __init__(self, rows):
        self.rows=rows
        assert(len(rows) == 2)
        assert(len(rows[0]) == 2)
        assert(len(rows[1]) == 2)
    @classmethod
    def rotation(cls, theta):
        return Mat2([
            [np.cos(theta),  -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ])
    def __mul__(self, vector):
        #  | a11 a12 |  
        #  | a21 a22 |  
        a11, a12 = self.rows[0]
        a21, a22 = self.rows[1]
        x = vector.x
        y = vector.y
        return Vec2(a11*x + a12*y, a21*x+a22*y)


class Poly2:
    def __init__(self, points):
        self.points = points


