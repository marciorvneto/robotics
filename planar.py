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
    def dot(self, other):
        return self.x*other.x + self.y*other.y
    def norm(self):
        return self.distTo(Point2(0,0))
    def rotate(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return Point2(
            c*self.x - s*self.y,
            s*self.x + c*self.y
        )
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


class State:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def wrap(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def distToSqr(self, other):
        return (self.x - other.x)**2 + (self.y - other.y)**2 + self.wrap(self.z - other.z)**2

    def distToSqrLambda(self, other, lamb=1.0):
        return (self.x - other.x)**2 + (self.y - other.y)**2 + lamb*self.wrap(self.z - other.z)**2

    def distToLambda(self, other, lamb=1.0):
        return np.sqrt(self.distToSqrLambda(other, lamb))

    def distTo(self, other):
        return np.sqrt(self.distToSqr(other))

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        """Returns the cross product vector, crucial for 3D SAT edge-edge axes."""
        return State(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def norm(self):
        return self.distTo(State(0, 0, 0))

    def rotateX(self, theta):
        """Rotates the vector around the X-axis."""
        c, s = np.cos(theta), np.sin(theta)
        return State(self.x, c * self.y - s * self.z, s * self.y + c * self.z)

    def rotateY(self, theta):
        """Rotates the vector around the Y-axis."""
        c, s = np.cos(theta), np.sin(theta)
        return State(c * self.x + s * self.z, self.y, -s * self.x + c * self.z)

    def rotateZ(self, theta):
        """Rotates the vector around the Z-axis (equivalent to 2D rotation)."""
        c, s = np.cos(theta), np.sin(theta)
        return State(c * self.x - s * self.y, s * self.x + c * self.y, self.z)

    def __sub__(self, other):
        return State(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return State(self.x + other.x, self.y + other.y, self.z + other.z)

    def __truediv__(self, scalar):
        if scalar == 0:
            raise ZeroDivisionError("Cannot divide a State by zero.")
        return State(self.x / scalar, self.y / scalar, self.z / scalar)

    def __mul__(self, scalar):
        return State(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def normalize(self):
        n = self.norm()
        if n == 0:
            return State(0, 0, 0)
        return State(self.x / n, self.y / n, self.z / n)

    def __repr__(self):
        return f"State({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


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


class GeneralRectangle:
    def __init__(self, state, w, h):
        self.x = state.x
        self.y = state.y
        self.theta = state.z
        self.w = w
        self.h = h

    @classmethod
    def from_rectangle(cls, r):
        return GeneralRectangle(State(r.x, r.y, 0.0), r.w, r.h)

    def normals(self):
        return [
            Point2(np.cos(self.theta), np.sin(self.theta)),
            Point2(-np.sin(self.theta), np.cos(self.theta)),
        ]
    def points(self):
        origin = Point2(self.x, self.y)

        local = [
            Point2(0, 0),
            Point2(self.w, 0),
            Point2(self.w, self.h),
            Point2(0, self.h),
        ]

        return [
            origin + p.rotate(self.theta)
            for p in local
        ]


class Path:
    def __init__(self, points):
        self.points = points


class Tree:
    def __init__(self, paths):
        self.paths = paths


def sat_rects(r1, r2):
    points_1 = r1.points()
    points_2 = r2.points()
    normals = [*r1.normals(), *r2.normals()]
    for n in normals:
        proj1 = [p.dot(n) for p in points_1]
        proj2 = [p.dot(n) for p in points_2]
        max1 = max(proj1)
        min1 = min(proj1)
        max2 = max(proj2)
        min2 = min(proj2)
        if max1 <= min2 or max2 <= min1:
            return False
    return True


class World:
    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.shapes = []
        self.paths = []
        self.trees = []
        self.start = None
        self.end = None
        self.bot = None

    def add_bot(self, bot):
        self.bot = bot
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

    def collides(self, state):
        # Assuming everything is a rectangle for now
        gr = GeneralRectangle(state, self.bot.w, self.bot.h)
        for shape in self.shapes:
            collision = sat_rects(gr, GeneralRectangle.from_rectangle(shape))
            if collision:
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

        if self.end:
            ax.plot(self.end.x, self.end.y, 'x', markersize=20, markeredgewidth=3, color='red')

        if self.bot:
            shape = self.bot
            rect_patch = patches.Rectangle(
                (shape.x, shape.y),
                shape.w,
                shape.h,
                angle=shape.theta * 180 / np.pi,
                linewidth=1,
                edgecolor="blue",
                facecolor="blue",
                alpha=0.3,
            )
            ax.add_patch(rect_patch)

        for tree in self.trees:
            for path in tree.paths:
                xs = [p.x for p in path]
                ys = [p.y for p in path]
                ax.plot(xs, ys, "-", linewidth=1, color="lightgrey")

        for path in self.paths:
            xs = [p.x for p in path.points]
            ys = [p.y for p in path.points]

            ax.plot(xs, ys, "-o", linewidth=2)

            # Robot footprint along trajectory
            for state in path.points:
                rect = patches.Rectangle(
                    (state.x, state.y),
                    self.bot.w,
                    self.bot.h,
                    angle=np.degrees(state.z),
                    linewidth=1,
                    fill=False,
                    alpha=0.3,
                )
                ax.add_patch(rect)

        plt.show()


@dataclass
class RRTParams:
    world: World
    start: State
    end: State
    tolerance: float
    max_step: float = 1e10
    max_iter: int = 7000
    p_goal: float = 0.25


class RRT:
    def __init__(self, params: RRTParams):
        self.params = params
        self.points = []
        self.parents = []
        
    def solve(self):
        closest = self.params.start.distTo(self.params.end)
        closest_point_idx = 0
        iters = 0

        self.points = [self.params.start]
        self.parents = [-1]
        
        while closest > self.params.tolerance and iters < self.params.max_iter:
            r = np.random.random()
            if r < self.params.p_goal:
                guide = self.params.end
            else:
                guide = self.gen_random()
                
            closest_idx = self.find_closest_idx(guide)
            candidate = self.steer(self.points[closest_idx], guide)
            collides = self.test_collision_along_path(self.points[closest_idx], candidate)
            
            if not collides:
                self.points.append(candidate)
                self.parents.append(closest_idx)
                dist_to_end = candidate.distTo(self.params.end)
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
        rtheta = np.random.random() * 2.0 * np.pi
        return State(rx, ry, rtheta)

    def steer(self, src, dest):
        delta = dest - src
        distance = delta.norm()

        if distance <= self.params.max_step:
            return dest

        return src + delta.normalize() * self.params.max_step

    def test_collision_along_path(self, src, dest, N=20):
        # Assuming src does not collide
        dv = (dest - src) / (1.0 * N)
        for i in range(1, N+1):
            v_test = src + i * dv
            if self.params.world.collides(v_test):
                return True
        return False

    def find_closest_idx(self, p):
        # O(n), might optimize later, but honestly... nah.
        best_dist2 = np.inf
        best_i = 0
        for i, other in enumerate(self.points):
            dist2 = p.distToSqrLambda(other)
            if dist2 < best_dist2:
                best_i = i
                best_dist2 = dist2
        return best_i


w = World(100, 100)

start = State(30, 70, 70 * np.pi/180)
end = State(75, 70, np.pi/2)
bot = GeneralRectangle(start, 15, 3)

w.add_bot(bot)
w.add_shape(Rectangle(40, 5, 20, 100))
w.add_shape(Rectangle(0, 0, 25, 30))
w.add_shape(Rectangle(0, -10, 100, 10))
w.add_start(start)
w.add_end(end)

rrt = RRT(RRTParams(
    world=w,
    start=start,
    end=end,
    tolerance=5.0,
    max_step=3.0
))

path = rrt.solve()
w.add_path(path)

tree = rrt.get_tree()
w.add_tree(tree)

w.render()
