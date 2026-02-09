import numpy as np
import matplotlib.pyplot as plt

class Plotter():
    def __init__(self, model, dataLog):
        self.model = model
        self.data = dataLog.data
        
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
        axs[1].set_ylabel('$Angular Velocity$')
    

        plt.xlabel('Time')

    def quadratic_plot_state(self, key: str, title='Quadratic Weighted State'):
        num_iterations = len(self.data[key])
        tspan = np.arange(0, num_iterations * self.model.mc.dt, self.model.mc.dt)
        fig, axs = plt.subplots(5)
        fig.set_figheight(8)
        fig.suptitle(title)

        for i in range(3):
            axs[0].plot(tspan, self.model.mc.Q[i]*self.data[key][:,i]**2)
        axs[0].set_ylabel('$x$')
        for i in range(3):
            axs[1].plot(tspan, self.model.mc.Q[i+3]*self.data[key][:,i+3]**2)
        axs[1].set_ylabel('$v$')
        for i in range(4):
            axs[2].plot(tspan, self.model.mc.Q[i+6]*self.data[key][:,i+6]**2)
        axs[2].set_ylabel('$q$')
        for i in range(3):
            axs[3].plot(tspan, self.model.mc.Q[i+10]*self.data[key][:,i+10]**2)
        axs[3].set_ylabel('$w$')
        for i in range(3):
            axs[4].plot(tspan, self.model.mc.Q[i+13]*self.data[key][:,i+13]**2)
        axs[4].set_ylabel('$a_ext$')
        plt.xlabel('Time')
