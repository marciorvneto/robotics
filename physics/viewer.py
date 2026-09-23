import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle as MplRectangle
import numpy as np

from geometry import *
from engine import *


class Viewer:
    def __init__(self, world, force_scale=0.1):
        self.world = world
        self.force_scale = force_scale

    def render_snapshot(self, xlim=None, ylim=None):
        fig, ax = plt.subplots()

        # Draw dynamic bodies
        for entt in self.world.dynamic:
            self._draw_entity(ax, entt)

        # Draw forces
        for force in self.world.forces:
            self._draw_force(ax, force)

        ax.set_aspect("equal")

        if xlim is not None:
            ax.set_xlim(*xlim)

        if ylim is not None:
            ax.set_ylim(*ylim)

        ax.grid(True)

        plt.show()

    def _draw_entity(self, ax, entt):
        if isinstance(entt, Rectangle):
            self._draw_rectangle(ax, entt)

    def _draw_rectangle(self, ax, body):
        # Matplotlib's Rectangle expects the lower-left corner,
        # but our body position is the center of mass.

        local_corner = Vec2(
            -body.w / 2,
            -body.h / 2,
        )

        world_corner = (
            body.pos
            + local_corner.rotate(body.theta)
        )

        patch = MplRectangle(
            (world_corner.x, world_corner.y),
            body.w,
            body.h,
            angle=np.degrees(body.theta),
            fill=False,
        )

        ax.add_patch(patch)

        # Center of mass
        ax.plot(
            body.pos.x,
            body.pos.y,
            "o",
        )

    def _draw_force(self, ax, force):
        body = force.entt

        # Resolve application point
        if force.position_frame == Frame.BODY:
            r = force.position.rotate(body.theta)
            p = body.pos + r
        else:
            p = force.position

        # Resolve force direction
        if force.direction_frame == Frame.BODY:
            direction = force.direction.rotate(body.theta)
        else:
            direction = force.direction

        F = direction * force.magnitude * self.force_scale

        ax.arrow(
            p.x,
            p.y,
            F.x,
            F.y,
            length_includes_head=True,
            head_width=0.05,
        )
