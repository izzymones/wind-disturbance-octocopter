from do_mpc.simulator import Simulator
from octa_model import OctaModel
from constants import Constants
import casadi as ca


class OctaSim():
    def __init__(self, model: OctaModel):
        self.model = model
        self.sim = Simulator(self.model.model)
        self.sim.x0 = ca.DM(model.mc.x0)
        self.sim.set_param(t_step=model.mc.dt)
        self.sim.setup()


    def step(self, u):
        x_next = self.sim.make_step(u)
        return x_next

