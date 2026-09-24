import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle as MplRectangle
from matplotlib.animation import FuncAnimation
import numpy as np

from geometry import *
from engine import *


class Viewer:
    def __init__(self, world, force_scale=0.1):
        self.world = world
        self.force_scale = force_scale

    def render_snapshot(self, xlim=None, ylim=None):
        fig, ax = plt.subplots()

        self._setup_axes(ax, xlim, ylim)
        self._draw_scene(ax)

        plt.show()

    def animate(
        self,
        simulator,
        T,
        xlim=None,
        ylim=None,
        interval=20,
    ):
        fig, ax = plt.subplots()

        nframes = int(T / simulator.dt)

        # step() currently expects these to exist
        simulator.accelerations = [
            Vec2(0, 0) for _ in self.world.dynamic
        ]
        simulator.ang_accelerations = [
            0.0 for _ in self.world.dynamic
        ]

        def update(frame):
            simulator.t = simulator.step(
                simulator.t,
                simulator.dt
            )

            ax.clear()

            self._setup_axes(ax, xlim, ylim)
            self._draw_scene(ax)

            ax.set_title(f"t = {simulator.t:.2f} s")

        ani = FuncAnimation(
            fig,
            update,
            frames=nframes,
            interval=interval,
            repeat=False,
        )

        plt.show()

        # Keep reference alive
        return ani

    def _setup_axes(self, ax, xlim, ylim):
        ax.set_aspect("equal")

        if xlim is not None:
            ax.set_xlim(*xlim)

        if ylim is not None:
            ax.set_ylim(*ylim)

        ax.grid(True)

    def _draw_scene(self, ax):
        for entt in self.world.static:
            self._draw_entity(ax, entt)

        for entt in self.world.dynamic:
            self._draw_entity(ax, entt)

        for force in self.world.forces:
            self._draw_force(ax, force)

    def _draw_entity(self, ax, entt):
        if isinstance(entt, Rectangle):
            self._draw_rectangle(ax, entt)
        if isinstance(entt, Plane):
            self._draw_plane(ax, entt)

    def _draw_rectangle(self, ax, body):
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

        ax.plot(
            body.pos.x,
            body.pos.y,
            "o",
        )

    def _draw_plane(self, ax, plane):
        n = plane.normal / plane.normal.norm()

        # Unit tangent
        tangent = Vec2(-n.y, n.x)

        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()

        # Only used to ensure the plane spans the viewport
        L = np.hypot(
            xmax - xmin,
            ymax - ymin,
        )

        p1 = plane.pos - tangent * L
        p2 = plane.pos + tangent * L

        ax.plot(
            [p1.x, p2.x],
            [p1.y, p2.y],
        )

        ax.plot(
            plane.pos.x,
            plane.pos.y,
            "o",
        )

        # Debug normal: fixed-ish visual size
        normal_length = 0.5

        ax.arrow(
            plane.pos.x,
            plane.pos.y,
            n.x * normal_length,
            n.y * normal_length,
            length_includes_head=True,
            head_width=0.08,
        )

    def _draw_force(self, ax, force):
        body = force.entt

        if force.position_frame == Frame.BODY:
            r = force.position.rotate(body.theta)
            p = body.pos + r
        else:
            p = force.position

        if force.direction_frame == Frame.BODY:
            direction = force.direction.rotate(body.theta)
        else:
            direction = force.direction

        F = (
            direction
            * force.magnitude
            * self.force_scale
        )

        ax.arrow(
            p.x,
            p.y,
            F.x,
            F.y,
            length_includes_head=True,
            head_width=0.05,
        )

    def animate_plan(
            self,
            simulator,
            set_state,
            get_q0,
            apply_u,
            us,
            num_steps,
            xlim=None,
            ylim=None,
            interval=20,
            ):
        schedule = []
        for u in us:
            if u is None:
                continue
            schedule.extend([u] * num_steps)

        # Roll the plan out once, independently of Matplotlib.  Animation
        # callbacks are not guaranteed to run exactly once per frame (the
        # first frame may be requested for initialization, for example), so
        # advancing the simulator from update() makes playback nondeterministic.
        set_state(simulator, simulator.world.dynamic[0], get_q0)
        if schedule:
            apply_u(schedule[0])

        snapshots = [self.world.get_snapshot()]
        times = [simulator.t]

        for u in schedule:
            apply_u(u)
            simulator.t = simulator.step(simulator.t, simulator.dt)
            snapshots.append(self.world.get_snapshot())
            times.append(simulator.t)

        # Leave the world at the start of the plan until frames are drawn.
        self.world.restore(snapshots[0])
        simulator.t = times[0]
        simulator.init_variables()

        fig, ax = plt.subplots()

        def draw_frame(frame):
            self.world.restore(snapshots[frame])
            simulator.t = times[frame]
            ax.clear()
            self._setup_axes(ax, xlim, ylim)
            self._draw_scene(ax)
            ax.set_title(f"t = {simulator.t:.2f} s")

        def init():
            draw_frame(0)

        def update(frame):
            draw_frame(frame)

        frames = range(1, len(snapshots)) if schedule else [0]
        ani = FuncAnimation(
                fig,
                update,
                frames=frames,
                init_func=init,
                interval=interval,
                repeat=False,
                )
        plt.show()
        return ani
