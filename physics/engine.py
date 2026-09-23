from enum import Enum, auto
from geometry import (
        Vec2
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
        self.forces = []
    def add_dynamic(self, entt):
        self.dynamic.append(entt)
        entt.on_append(self)
    def add_force(self, f):
        self.forces.append(f)


class Rectangle:
    def __init__(self, m, w, h, pos=Vec2(0,0), theta=0):
        self.w = w
        self.h = h
        self.m = m
        self.Icm = 1.0/12 * m * (w * w + h * h)
        self.pos = pos
        self.theta = theta

    def on_append(self, w):
        w.add_force(Force(self, self.m * G, Vec2(0,0), Vec2(0, -1.0), direction_frame=Frame.WORLD))
