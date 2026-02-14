import casadi as ca
import do_mpc

class OctoModel:
    def __init__(self, mc):
        self.mc = mc

        self.model = do_mpc.model.Model('continuous' , 'SX')
        p = self.model.set_variable(var_type='_x', var_name='p', shape=(3,1))
        v = self.model.set_variable(var_type='_x', var_name='v', shape=(3,1))
        q = self.model.set_variable(var_type='_x', var_name='q', shape=(4,1))
        w = self.model.set_variable(var_type='_x', var_name='w', shape=(3,1))
        a_ext = self.model.set_variable(var_type='_x', var_name='a_ext', shape=(3,1))

        state = ca.vertcat(p,v,q,w,a_ext)
        u = self.model.set_variable(var_type='_u', var_name='u', shape=(4,1))

        I_mat = ca.DM(mc.I)

        T_vector = ca.vertcat(0,0,u[0])

        a_ext_vector = ca.vertcat(0,0,0)

        M_vector = ca.vertcat(u[1],u[2],u[3])
        angular_momentum = I_mat @ w

        q_full = state[6:10]
        q_full = q_full / ca.norm_2(q_full)

        r_b2w = ca.vertcat(
            ca.horzcat(
                1 - 2*(q_full[1]**2 + q_full[2]**2), 
                2*(q_full[0]*q_full[1] - q_full[2]*q_full[3]), 
                2*(q_full[0]*q_full[2] + q_full[1]*q_full[3])
                ),
            ca.horzcat(
                2*(q_full[0]*q_full[1] + q_full[2]*q_full[3]), 
                1 - 2*(q_full[0]**2 + q_full[2]**2), 
                2*(q_full[1]*q_full[2] - q_full[0]*q_full[3])
                ),
            ca.horzcat(
                2*(q_full[0]*q_full[2] - q_full[1]*q_full[3]),
                2*(q_full[1]*q_full[2] + q_full[0]*q_full[3]), 
                1 - 2*(q_full[0]**2 + q_full[1]**2)
                ),
        )

        Q_omega = ca.vertcat(
            ca.horzcat(0, state[12], -state[11], state[10]),
            ca.horzcat(-state[12], 0, state[10], state[11]),
            ca.horzcat(state[11], -state[10], 0, state[12]),
            ca.horzcat(-state[10], -state[11], -state[12], 0)
        )

        self.model.set_rhs('p', v)
        self.model.set_rhs('v', (r_b2w @ T_vector) / mc.m + mc.g + a_ext)
        self.model.set_rhs('q', 0.5 * Q_omega @ q_full)
        self.model.set_rhs('w', ca.solve(I_mat, M_vector - ca.cross(w, angular_momentum)))
        self.model.set_rhs('a_ext', ca.SX(3,1))


        x_r = ca.vertcat(mc.xr)
        x_error = state - x_r
        x_cost = x_error.T @ mc.Q @ x_error 
        terminal_cost = x_error.T @ (mc.terminal_cost_factor * mc.Q) @ x_error 
        u_goal = ca.DM([-mc.gz * mc.m, 0.0, 0.0, 0.0])
        u_error = u - u_goal
        u_cost = u_error.T @ mc.R @ u_error
        cost = x_cost + u_cost
        self.model.set_expression(expr_name='terminal_cost', expr=terminal_cost )
        self.model.set_expression(expr_name='cost', expr=cost)

        self.model.setup()