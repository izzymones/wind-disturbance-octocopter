import numpy as np
import matplotlib.pyplot as plt

class Plotter():
    def __init__(self, model):
        self.model = model
        self.data = {}

    def allocate_data(self, key: str, iterations: int, length: int):
        self.data[key] = np.empty([iterations, length], dtype=float)

    def add_point(self, key: str, k: int, array):
        arr = np.asarray(array, dtype=float).reshape(-1)
        self.data[key][k] = arr
        
    def plot_state(self, key: str, title='State'):
        num_iterations = len(self.data[key])
        tspan = np.arange(0, num_iterations * self.model.mc.dt, self.model.mc.dt)
        fig, axs = plt.subplots(5)
        fig.set_figheight(8)
        fig.suptitle(title)

        for i in range(3):
            axs[0].plot(tspan, self.data[key][:,i])
        axs[0].set_ylabel('$x$')
        for i in range(3):
            axs[1].plot(tspan, self.data[key][:,i+3])
        axs[1].set_ylabel('$v$')
        for i in range(4):
            axs[2].plot(tspan, self.data[key][:,i+6])
        axs[2].set_ylabel('$q$')
        for i in range(3):
            axs[3].plot(tspan, self.data[key][:,i+10])
        axs[3].set_ylabel('$w$')
        for i in range(3):
            axs[4].plot(tspan, self.data[key][:,i+13])
        axs[4].set_ylabel('$a_ext$')
        plt.xlabel('Time')

    def plot_control(self, key: str, title='Control'):
        num_iterations = len(self.data[key])
        tspan = np.arange(0, num_iterations * self.model.mc.dt, self.model.mc.dt)
        plt.rcParams['ytick.labelsize'] = 8 
        plt.rcParams['xtick.labelsize'] = 8
        fig, axs = plt.subplots(2)
        fig.set_figheight(8)
        fig.suptitle(title)

        axs[0].plot(tspan, self.data[key][:,0])
        axs[0].set_ylabel('$Thrust$')

        for i in range(3):
            axs[1].plot(tspan, self.data[key][:,i+1])
        axs[1].set_ylabel('$Torque$')
    

        plt.xlabel('Time')
