import numpy as np
import casadi as ca

class Constants:
    def __init__(self):

        self.dt = 0.02
        self.sim_seconds = 10
        self.num_iterations = int(round(self.sim_seconds / self.dt))

        self.m = 5

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

        self.I_diag = np.array([self.Ixx, self.Iyy, self.Izz])
        self.I_inv = np.linalg.inv(self.I)

        self.dt = 0.02
        # self.x0 = ca.vertcat(0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.707,0.707, 0.0,0.0,0.0)
        self.x0 = [1.0,2.0,3.0, 2.0,3.0,4.0, 0.308,-0.329,0.379,0.808, np.pi*50.0 / 180,np.pi*5.0 / 180,np.pi*5.0 / 180, 0.0,0.0,0.0]
        self.xr = self.x0

        self.Q = ca.diag([20.0,20.0,20.0,10.0,10.0,10.0,500.0,500.0,500.0,0.0,15.0,15.0,10.0,0.25,0.25,0.25])
        self.R = ca.diag([1.0, 10.0, 10.0, 10.0])

        self.terminal_cost_factor = 5.0

        self.L = 0.5
        self.prop_rad = 0.07

        self.beta = 0.2
        self.time_constant = 0.15

        self.kd = self.beta * self.I_diag
        self.kp = (self.I_diag + self.kd) / self.time_constant


        self.tau_max = np.array([3.0, 3.0, 3.0])

        self.ipopt_settings = {
            "ipopt.max_iter": 100,                   
            "ipopt.tol": 1e-3,                     
            "ipopt.acceptable_tol": 1e-4,
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0,
            'ipopt.linear_solver': 'mumps',
            # 'ipopt.warm_start_init_point': 'yes',
            # 'ipopt.warm_start_bound_push': 1e-6,
            # 'ipopt.warm_start_mult_bound_push': 1e-6,
            # 'ipopt.mu_init': 1e-3,  
        }

        self.finite_interval_size = 0.3
        self.number_intervals = 6
        self.collocation_degree = 2