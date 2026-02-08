from octa_model import OctaModel
from constants import Constants
import casadi as ca
import numpy as np
from simulator import OctaSim
from visualizer import Visualizer
from plotter import Plotter

import matplotlib.pyplot as plt


mc = Constants()
model = OctaModel(mc)
simulator = OctaSim(model)
visualizer = Visualizer(model)
plotter = Plotter(model)

num_iterations = 200

u0 = np.array(
    [
        [-mc.m * mc.gz],
        [0.1],
        [0.1],
        [0.1],
    ],
    dtype=float,
)

plotter.allocate_data("state", num_iterations, 16)
plotter.allocate_data("control", num_iterations, 4)

for k in range(num_iterations):
    x = simulator.step(u0)

    plotter.add_point("state", k, x)
    plotter.add_point("control", k, u0)

    visualizer.update(x[0:3], x[6:10])

    print(x, k * mc.dt)

plotter.plot_state("state")
plotter.plot_control("control")

plt.ioff()
plt.show()
