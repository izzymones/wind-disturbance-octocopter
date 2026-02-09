import numpy as np

class PX4RateLoopEmulator():
    def __init__(self, model):
        self.model = model
        self.dt = float(self.model.mc.dt)
        self.kp = np.asarray(self.model.mc.kp, dtype=float).reshape(3,)

        self.kd = np.asarray(self.model.mc.kd, dtype=float).reshape(3,)

        self.tau_max = np.asarray(self.model.mc.tau_max, dtype=float).reshape(3,)

        self.old_w = None
    
    def step(self, u_cmd, w_meas):
        u_cmd = np.array(u_cmd, dtype=float).reshape(-1)

        T_cmd = u_cmd[0]
        w_cmd = u_cmd[1:4]

        w_meas = np.array(w_meas, dtype=float).reshape(3,)

        error = w_cmd - w_meas

        if self.old_w is None:
            w_dot = np.zeros(3)
        else:
            w_dot = (w_meas - self.old_w) / self.dt
        self.old_w = w_meas.copy()

        tau_cmd = self.kp * error - self.kd * w_dot
        tau_cmd = np.clip(tau_cmd, -self.tau_max, self.tau_max)

        return np.array([T_cmd, tau_cmd[0], tau_cmd[1], tau_cmd[2]], dtype=float).reshape(4, 1)