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
    def __repr__(self):
        return f"({self.x:.4f},{self.y:.4f})"

class Poly2:
    def __init__(self, points):
        self.points = points
