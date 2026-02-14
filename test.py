import casadi as ca
import numpy as np
from constants import Constants
from octo_model import OctoModel
from emulator import PX4RateLoopEmulator
from simulator import OctoSim
from visualizer import Visualizer
from plotter import Plotter
from data import DataLog

from nmpc import OctoNMPCdompc

import matplotlib.pyplot as plt


mc = Constants()
model = OctoModel(mc)
mpc = OctoNMPCdompc(mc.dt, model.model)
emulator = PX4RateLoopEmulator(model)
dataLog = DataLog()
simulator = OctoSim(model)
plotter = Plotter(model, dataLog)
visualizer = Visualizer(model, dataLog)


x = ca.DM(mc.x0)

mpc.setup_cost()
mpc.set_start_state(x)


dataLog.allocate_data("state", mc.num_iterations, 16)
dataLog.allocate_data("control", mc.num_iterations, 4)

for k in range(mc.num_iterations):
    # y = randomizer(x)
    # x = estimator(y)
    u = mpc.mpc.make_step(x)
    print(x,u)

    u_torque = emulator.step(u, x[10:13])
    x = simulator.step(u)

    dataLog.add_point("state", k, x)
    dataLog.add_point("control", k, u)

    # visualizer.update(x[0:3], x[6:10])

visualizer.print_trajectory()
visualizer.print_trajectory()
visualizer.print_trajectory()

plotter.plot_state("state")
# plotter.quadratic_plot_state("state")
plotter.plot_control("control")

plt.ioff()
plt.show()
