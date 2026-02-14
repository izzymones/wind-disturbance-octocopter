import numpy as np
import casadi as ca

class Constants:
    def __init__(self):

        # divide and rounds dt by simulation time to get number of iterations

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

        # inertia tensor
        self.I = np.array([
            [self.Ixx,0.0,0.0],
            [0.0,self.Iyy,0.0],
            [0.0,0.0,self.Izz]
        ])

        # inertia diagonal
        self.I_diag = np.array([self.Ixx, self.Iyy, self.Izz])
        self.I_inv = np.linalg.inv(self.I)

                # test initial state
        # state: 3 pos, 3 vel, 4 quaternion, 3 angular velocity, 3 disturbance force vector (not yet in use)

        # self.x0 = [4.0,5.0,0.0, 0.0,5.0,0.0, 0.0,0.0,0.0,1.0, np.pi*0.0 / 180,np.pi*0.0 / 180,np.pi*0.0 / 180, 0.0,0.0,0.0]
        # self.x0 = [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,1.0, np.pi*45.0 / 180,np.pi*45.0 / 180,np.pi*.0 / 180, 0.0,0.0,0.0]
        self.x0 = [0.0,0.0,0.0, 0.0,0.0,0.0, 0.230,0.159,0.186,0.942, np.pi*0.0 / 180,np.pi*0.0 / 180,np.pi*0.0 / 180, 0.0,0.0,0.0]
        self.xr = [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0]

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