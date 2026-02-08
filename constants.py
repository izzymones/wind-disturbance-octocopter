import numpy as np
import casadi as ca

class Constants:
    def __init__(self):

        self.dt = 0.02
        self.m = 5

        self.px4_height = 0.3

        self.gx = 0
        self.gy = 0
        self.gz = -9.81
        self.g = np.array([
            self.gx,
            self.gy,
            self.gz
        ])

        self.Ixx = 0.01
        self.Iyy = 0.01
        self.Izz = 0.01


        self.I = np.array([
            [self.Ixx,0.0,0.0],
            [0.0,self.Iyy,0.0],
            [0.0,0.0,self.Izz]
        ])

        self.I_diag = [self.Ixx, self.Iyy, self.Izz]
        self.I_inv = np.linalg.inv(self.I)

        self.dt = 0.02
        # self.x0 = ca.vertcat(0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.707,0.707, 0.0,0.0,0.0)
        self.x0 = [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0]
        self.xr = self.x0

        self.Q = ca.diag([1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,1.0,1.0,1.0,1.0,1.0,1.0])
        self.R = ca.diag([1, 1, 1, 1])

        self.terminal_cost_factor = 0.0

        self.L = 0.5
        self.prop_rad = 0.07