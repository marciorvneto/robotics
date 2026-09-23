import numpy as np
from geometry import *
from engine import *
from viewer import *

w = World()

drone = Rectangle(1.0, 0.3, 0.1, pos=Vec2(5,2), theta=np.radians(10))

thruster_force = 5.0

thruster_left  = Force(drone,
                       thruster_force,
                       Vec2(-0.15, 0),
                       Vec2(0, 1.0)
                       )

thruster_right = Force(drone,
                       thruster_force,
                       Vec2(0.15, 0),
                       Vec2(0, 1.0)
                       )

w.add_dynamic(drone)
w.add_force(thruster_left)
w.add_force(thruster_right)

viewer = Viewer(w)
viewer.render_snapshot(xlim=(0,10), ylim=(0,6))
