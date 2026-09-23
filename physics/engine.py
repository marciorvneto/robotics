import numpy as np
from enum import Enum, auto
from geometry import (
        Vec2,
        Mat2,
        )

G = 9.81 # Gravity

class Frame(Enum):
    WORLD = auto()
    BODY  = auto()

class Force:
    def __init__(
            self, 
            entt, 
            magnitude, 
            position, 
            direction,
            position_frame=Frame.BODY,
            direction_frame=Frame.BODY,
            ):
        """Positions here are given wrt the CM"""
        self.entt = entt
        self.direction = direction
        self.position = position
        self.magnitude = magnitude
        self.position_frame = position_frame
        self.direction_frame = direction_frame

class World:
    def __init__(self):
        self.dynamic = []
        self.static = []
        self.forces = []
    def add_dynamic(self, entt):
        self.dynamic.append(entt)
        entt.on_append(self)
    def add_static(self, entt):
        self.static.append(entt)
        entt.on_append(self)
    def add_force(self, f):
        self.forces.append(f)


class Rectangle:
    def __init__(self, m, w, h, pos=Vec2(0,0), vel=Vec2(0,0), theta=0, ang_vel=0):
        self.w = w
        self.h = h
        self.m = m
        self.Icm = 1.0/12 * m * (w * w + h * h)
        self.pos = pos
        self.vel = vel
        self.ang_vel = ang_vel
        self.theta = theta

        self.total_force = Vec2(0,0)
        self.total_torque = 0

    def on_append(self, w):
        w.add_force(Force(self, self.m * G, Vec2(0,0), Vec2(0, -1.0), direction_frame=Frame.WORLD))
    def get_frame(self):
        return Mat2.rotation(self.theta)

    def get_points_world(self):
        points_body = [
                Vec2(-self.w/2, -self.h/2),
                Vec2(-self.w/2, self.h/2),
                Vec2(self.w/2, self.h/2),
                Vec2(self.w/2, -self.h/2),
                ]
        R = Mat2.rotation(self.theta)
        points_world = [(R * p) + self.pos for p in points_body]
        return points_world

    def __repr__(self):
        angle = np.degrees(self.theta)

class Plane:
    def __init__(self, pos=Vec2(0,0), normal=Vec2(0,1)):
        self.pos = pos
        self.normal = normal.normalize()

    def on_append(self, w):
        pass

    def get_frame(self):
        return None

    def __repr__(self):
        return f"[Plane] ({self.pos.x:.2f},{self.pos.y:.2f},{self.angle.x:.2f},{self.angle.y:.2f},)"


class Contact:
    def __init__(self, body, point, normal, depth):
        self.body   = body
        self.point  = point
        self.normal = normal
        self.depth  = depth

class CollisionFunctions:
    @classmethod
    def rect_plane(cls, rect, plane):
        """Assumes COM coordinates"""
        rect_points = rect.get_points_world()

        contacts = []

        for p in rect_points:
            signed_depth = (p - plane.pos).dot(plane.normal)

            if signed_depth < 0:

                contact_point = p - signed_depth * plane.normal

                contacts.append(Contact(
                    body   = rect,
                    point  = contact_point,
                    normal = plane.normal,
                    depth  = np.abs(signed_depth)
                    ))

        return contacts


class Simulator:
    def __init__(self, world, dt=0.001):
        self.world = world
        self.dt = dt
        self.t = 0
        self.accelerations     = []
        self.ang_accelerations = []

    def simulate(self, T):
        self.accelerations     = [Vec2(0,0) for e in self.world.dynamic]
        self.ang_accelerations = [0 for e in self.world.dynamic]
        while self.t <= T:
            self.t = self.step(self.t, self.dt)
            print(self.world.dynamic[0])

    def step(self, t, dt):
        for entt in self.world.dynamic:
            entt.total_force = Vec2(0,0)
            entt.total_torque = 0

        # Verlet - position bump

        self.apply_forces()

        for i,entt in enumerate(self.world.dynamic):
            a                         = entt.total_force / entt.m
            aang                      = entt.total_torque / entt.Icm
            self.accelerations[i]     = a
            self.ang_accelerations[i] = aang
            entt.pos                  = entt.pos + entt.vel * dt + a * dt * dt / 2
            entt.theta                = (entt.theta + entt.ang_vel * dt + aang * dt * dt / 2) % (2 * np.pi)

        # Verlet - velocity bump

        self.clear_forces()
        self.apply_forces()

        for i,entt in enumerate(self.world.dynamic):
            a         = entt.total_force / entt.m
            aang      = entt.total_torque / entt.Icm
            a_old     = self.accelerations[i]
            a_ang_old = self.ang_accelerations[i]
            entt.vel     += (a + a_old)/2 * dt
            entt.ang_vel += (a_ang_old + aang)/2 * dt

        # Check collisions
        # Keep it ultra simple for now, only checking with planes
        
        if len(self.world.static) > 0:
            contacts = []
            for plane in self.world.static:
                for entt in self.world.dynamic:
                    # Assuming we only have rects
                    these_contacts = CollisionFunctions.rect_plane(entt, plane)
                    for c in these_contacts:
                        contacts.append(c)

            for c in contacts:
                self.resolve_contact(c)

        return t + dt

    def resolve_contact(self, c):
        body  = c.body
        point = c.point
        n     = c.normal
        depth = c.depth

        arm      = point - body.pos
        tang_vel = body.ang_vel * Vec2(-arm.y, arm.x)
        vel      = tang_vel + body.vel
        vel_n    = vel.dot(n)

        if vel_n >= 0:
            # Moving away from contact
            return

        e   = 0.3
        m   = body.m
        Icm = body.Icm
        rn  = arm.cross(n)
        j   = -(1 + e) * vel_n / (1/m + rn**2/Icm)

        impulse       = j * n
        body.vel     += impulse / m
        body.ang_vel += arm.cross(impulse) / Icm



    def apply_force(self, f):
        entt = f.entt
        R = entt.get_frame()

        if f.direction_frame==Frame.BODY:
            direction_world = R * f.direction
        else:
            direction_world = f.direction

        f_world = f.magnitude * direction_world

        if f.position_frame==Frame.BODY:
            r_world = R * f.position
        else:
            r_world = f.position - entt.pos

        t_world = r_world.cross(f_world)

        entt.total_force += f_world
        entt.total_torque += t_world

    def clear_forces(self):
        for entt in self.world.dynamic:
            entt.total_force = Vec2(0, 0)
            entt.total_torque = 0.0

    def apply_forces(self):
        for f in self.world.forces:
            self.apply_force(f)























