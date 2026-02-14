from do_mpc.simulator import Simulator
from octo_model import OctoModel
import casadi as ca


class OctoSim():
    def __init__(self, model: OctoModel):
        self.model = model
        self.sim = Simulator(self.model.model)
        self.sim.x0 = ca.DM(model.mc.x0)
        self.sim.set_param(t_step=model.mc.dt)
        self.sim.setup()


    def step(self, u):
        x_next = self.sim.make_step(u)
        return x_next

