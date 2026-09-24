import numpy as np
from geometry import *
from engine import *
from viewer import *

w = World()

floor = Plane(pos=Vec2(2,0.2), normal=Vec2(0,1))
slope = Plane(pos=Vec2(6,0.2), normal=Vec2(-1,1))
drone = Rectangle(1.0, 0.3, 0.1, pos=Vec2(5,2), theta=np.radians(0))

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
w.add_static(floor)
w.add_static(slope)
w.add_force(thruster_left)
w.add_force(thruster_right)

viewer = Viewer(w, force_scale=0.05)

simulator = Simulator(w, 0.01)

fmin, fmax = 0, 20
# W = np.array([1.0, 1.0, 0.15, 0.15, 0.4, 0.05])
W = np.array([1.0, 1.0, 1.0, 0.15, 0.15, 0.05])


class DynRRT:
    def __init__(self,
                 simulator,
                 agent,
                 end_pos,
                 max_iter=5000,
                 num_steps=5,
                 tol=0.1):

        self.simulator     = simulator
        self.agent         = agent
        self.max_iter      = max_iter
        self.num_steps     = num_steps
        self.tol           = tol
        self.end_pos       = end_pos
        self.initial_state = w.get_snapshot()
        simulator.reset(self.initial_state)

    def get_state(self, agent):
        return np.array([
            agent.pos.x,
            agent.pos.y,
            agent.theta,
            agent.vel.x,
            agent.vel.y,
            agent.ang_vel,
            ], dtype=np.float64)

    def set_state(self, sim, agent, state):
        x, y, theta, velx, vely, ang_vel = state
        agent.pos = Vec2(x,y)
        agent.vel = Vec2(velx,vely)
        agent.theta = theta
        agent.ang_vel = ang_vel
        sim.t = 0
        sim.init_variables()

    def wrap_angle(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def distance(self, q1, q2):
        d = q1 - q2
        d[2] = self.wrap_angle(d[2])
        return np.linalg.norm(W * d)

    def sample_q(self, q_end, p_goal=0.2):
        if np.random.random() < p_goal:
            return q_end
        return np.array([
            np.random.uniform(0, 10),
            np.random.uniform(0, 10),
            np.random.uniform(-np.pi, np.pi),
            np.random.uniform(-1, 1),
            np.random.uniform(-1, 1),
            np.random.uniform(-2,2),
            ], dtype=np.float64)

    def sample_u(self):
        return np.random.random(2) * (fmax - fmin) + fmin

    def apply_u(self, u):
        thruster_left.magnitude = float(u[0])
        thruster_right.magnitude = float(u[1])

    def solve(self):
        qs               = [self.get_state(self.agent)]
        parents          = [-1]
        us               = [None]
        q_end            = [self.end_pos.x, self.end_pos.y, 0, 0, 0, 0]
        best_dist_to_end = self.distance(qs[0], q_end)
        closest_q_i      = 0

        for _ in range(self.max_iter):
            q_rand = self.sample_q(q_end)

            # Get nearest q
            i_near = 0
            best_distance = np.inf
            for i,q in enumerate(qs):
                d = self.distance(q_rand, q) 
                if d < best_distance:
                    i_near        = i
                    best_distance = d

            u     = self.sample_u()
            q_new = self.steer(qs[i_near], u)

            if q_new is None:
                continue

            qs.append(q_new)
            us.append(u)
            parents.append(i_near)

            dist = self.distance(q_new, q_end)
            if dist < best_dist_to_end:
                best_dist_to_end, closest_q_i = dist, len(qs) - 1
                print(f"Best: {best_dist_to_end:.2f}")

            if dist < self.tol:
                print(f"Converged! dist={dist:.2f}")
                break

        return self.trace_path(qs, us, parents, closest_q_i)



    def trace_path(self, qs, us, parents, closest_i):
        q_path = [qs[closest_i]]
        u_path = [us[closest_i]]
        parent = parents[closest_i]

        while parent > -1:
            q_path.append(qs[parent])
            u_path.append(us[parent])
            parent = parents[parent]

        return q_path[::-1], u_path[::-1]

    def steer(self, q, u):
        self.set_state(self.simulator, self.agent, q)
        self.apply_u(u)
        for _ in range(self.num_steps):
            self.simulator.step(self.simulator.t, self.simulator.dt)
            if self.simulator.collided:
                return None
        return self.get_state(self.agent)


rrt = DynRRT(simulator, drone, Vec2(7,8), tol=0.5)
qs, us = rrt.solve()

viewer = Viewer(w, force_scale=0.05)

ani = viewer.animate_plan(
    simulator,
    set_state=rrt.set_state,
    get_q0=qs[0],
    apply_u=rrt.apply_u,
    us=us,
    num_steps=rrt.num_steps,
    xlim=(0, 10),
    ylim=(0, 10),
)
