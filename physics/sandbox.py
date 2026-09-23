import numpy as np
from geometry import *
from engine import *
from viewer import *

w = World()

floor = Plane(pos=Vec2(2,0.2), normal=Vec2(0,1))
slope = Plane(pos=Vec2(6,0.2), normal=Vec2(-1,1))
drone = Rectangle(1.0, 0.3, 0.1, pos=Vec2(5,2), theta=np.radians(10))

thruster_force = 5.0

thruster_left  = Force(drone,
                       thruster_force,
                       Vec2(-0.15, 0),
                       Vec2(0, 1.0)
                       )

thruster_right = Force(drone,
                       thruster_force*0.97,
                       Vec2(0.15, 0),
                       Vec2(0, 1.0)
                       )

w.add_dynamic(drone)
w.add_static(floor)
w.add_static(slope)
w.add_force(thruster_left)
w.add_force(thruster_right)

viewer = Viewer(w, force_scale=0.05)

simulator = Simulator(w, 0.01)

ani = viewer.animate(
    simulator,
    T=5.0,
    xlim=(0, 10),
    ylim=(0, 10),
    interval=10,
)
