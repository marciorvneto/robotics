import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from dataclasses import dataclass


class Point2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def distToSqr(self, other):
        return (self.x - other.x)**2 + (self.y - other.y)**2
    def distTo(self, other):
        return np.sqrt(self.distToSqr(other))
    def norm(self):
        return self.distTo(Point2(0,0))
    def __sub__(self, other):
        return Point2(self.x - other.x, self.y - other.y)
    def __add__(self, other):
        return Point2(self.x + other.x, self.y + other.y)
    def __truediv__(self, scalar):
        if scalar == 0:
            raise ZeroDivisionError("Cannot divide a Point2 by zero.")
        return Point2(self.x / scalar, self.y / scalar)
    def __mul__(self, scalar):
        return Point2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    def normalize(self):
        norm = self.distTo(Point2(0,0))
        return Point2(self.x/norm, self.y/norm)
    def __repr__(self):
        return f"Point2({self.x:.2f}, {self.y:.2f})"


class Rectangle:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, p):
        if p.x < self.x or p.x > self.x + self.w:
            return False
        if p.y < self.y or p.y > self.y + self.h:
            return False
        return True

class Path:
    def __init__(self, points):
        self.points = points

class Tree:
    def __init__(self, paths):
        self.paths = paths



class World:
    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.shapes = []
        self.paths = []
        self.trees = []
        self.start = None
        self.end = None

    def add_shape(self, shape):
        self.shapes.append(shape)
    def add_path(self, path):
        self.paths.append(Path(path))
    def add_tree(self, tree):
        self.trees.append(Tree(tree))
    def add_start(self, start):
        self.start = start
    def add_end(self, end):
        self.end = end

    def collides(self, p):
        for shape in self.shapes:
            if shape.contains(p):
                return True
        return False

    def render(self):
        fig, ax = plt.subplots(figsize=(6, 6))

        ax.set_xlim(0, self.w)
        ax.set_ylim(0, self.h)
        ax.set_aspect("equal")
        ax.grid(True, linestyle="--", alpha=0.5)

        for shape in self.shapes:
            if isinstance(shape, Rectangle):
                rect_patch = patches.Rectangle(
                    (shape.x, shape.y),
                    shape.w,
                    shape.h,
                    linewidth=1,
                    edgecolor="red",
                    facecolor="red",
                    alpha=0.3,
                )
                ax.add_patch(rect_patch)

        if self.start:
            ax.plot(self.start.x, self.start.y, 's', markersize=20, markeredgewidth=3, color='green')

        if self.end:
            ax.plot(self.end.x, self.end.y, 'x', markersize=20, markeredgewidth=3, color='red')

        for tree in self.trees:
            for path in tree.paths:
                xs = [p.x for p in path]
                ys = [p.y for p in path]
                ax.plot(xs, ys, "-", linewidth=1, color="lightgrey")

        for path in self.paths:
            xs = [p.x for p in path.points]
            ys = [p.y for p in path.points]

            ax.plot(xs, ys, "-o", linewidth=2)

        plt.show()

@dataclass
class RRTParams:
    world: World
    start: Point2
    end: Point2
    tolerance: float
    max_step: float = 1e10
    max_iter: int = 2000

class RRT:
    def __init__(self, params: RRTParams):
        self.params = params
        self.points = []
        self.parents = []
    def solve(self):
        self.closest_point_idx = None
        closest = 1e10
        iters = 0
        converged = False
        self.points = [self.params.start]
        self.parents = [-1]
        while closest > self.params.tolerance and iters < self.params.max_iter:
            new_candidate = self.gen_random()
            closest_idx = self.find_closest_idx(new_candidate)
            new_free = self.steer(self.points[closest_idx],new_candidate)
            new_free = self.test_collision_along_path(self.points[closest_idx],new_candidate)
            if new_free:
                self.points.append(new_free)
                self.parents.append(closest_idx)
                dist_to_end = new_free.distTo(self.params.end)
                if dist_to_end < closest:
                    closest = dist_to_end
                    closest_point_idx = len(self.points) - 1
            iters += 1

        # Assuming it converged, roll out the path by tracing back from the parents list

        if closest < self.params.tolerance:
            print("Converged!")
        else:
            print("Did not converge")

        rev_path = []
        point_idx = closest_point_idx
        while self.parents[point_idx] >= 0:
            rev_path.append(self.points[point_idx])
            point_idx = self.parents[point_idx]
        # Now, push the first point
        rev_path.append(self.points[0])
        path = rev_path[::-1]
        return path
        

    def get_tree(self):
        paths = []

        for i in range(1, len(self.points)):
            parent_idx = self.parents[i]
            paths.append([
                self.points[parent_idx],
                self.points[i]
            ])

        return paths
                

    def gen_random(self):
        rx = np.random.random() * self.params.world.w
        ry = np.random.random() * self.params.world.h
        return Point2(rx, ry)

    def steer(self, src, dest):
        delta = dest - src
        distance = delta.norm()

        if distance <= self.prams.max_step:
            return dest

        return src + delta.normalize() * self.params.max_step

    def test_collision_along_path(self, src, dest, N=20):
        # Assuming src does not collide
        dv = (dest - src) / (1.0 * N)
        dv_len = dv.norm()
        last_valid = src
        for i in range(1,N+1):
            v_test = src + i * dv
            if(self.params.world.collides(v_test)) or i * dv_len >= self.params.max_step:
                if i == 1:
                    return None
                return last_valid
            last_valid = v_test
        return last_valid


    def find_closest_idx(self, p):
        # O(n2), might optimize later, but honestly... nah.
        best_dist2 = np.inf
        best_i = 0
        for i, other in enumerate(self.points):
            dist2 = p.distToSqr(other)
            if dist2 < best_dist2:
                best_i = i
                best_dist2 = dist2
        return best_i




w = World(100, 100)
start = Point2(30, 30)
end = Point2(70, 70)

w.add_shape(Rectangle(0, 0, 20, 30))
w.add_shape(Rectangle(50, 50, 20, 30))
w.add_start(start)
w.add_end(end)

rrt = RRT(RRTParams(
    world=w,
    start=start,
    end=end,
    tolerance=1.0,
    max_step=3.0
    ))

path = rrt.solve()
w.add_path(path)

tree = rrt.get_tree()
print(tree)
w.add_tree(tree)

w.render()
