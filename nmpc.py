import do_mpc
import numpy as np
import casadi as ca

from constants import Constants

mc = Constants()

class OctoNMPCdompc:
    def __init__(self, dt, model):
        self.dt = dt
        self.model = model
        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.settings.nlpsol_opts = mc.ipopt_settings
        self.mpc.settings.collocation_ni = mc.collocation_degree
        self.mpc.settings.t_step = mc.finite_interval_size    
        self.mpc.settings.n_horizon = mc.number_intervals
        self.mpc.settings.collocation_deg = mc.collocation_degree  

        # only do this if we need the stats
        self.mpc.settings.store_full_solution = True

        # upper and lower bounds on position (x,y,z)
        self.mpc.bounds['lower', '_x', 'p'] = [-ca.inf, -ca.inf, 0]
        self.mpc.bounds['upper', '_x', 'p'] = [ ca.inf,  ca.inf,  ca.inf]


    def set_start_state(self, x0):
        self.mpc.x0 = x0
        self.mpc.set_initial_guess()


    def setup_cost(self):
        self.mpc.set_objective(mterm=self.model.aux['terminal_cost'], lterm=self.model.aux['cost'])
        # self.mpc.set_rterm(u=mc.actuator_rate_costs, dtype=float)
        self.mpc.setup()