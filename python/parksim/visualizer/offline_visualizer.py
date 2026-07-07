from dataclasses import dataclass
import numpy as np

import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation, PillowWriter

from parksim.pytypes import VehiclePrediction
from parksim.obstacle_types import GeofenceRegion
from parksim.vehicle_types import VehicleBody

def plot_vehicle(ax, state, steer, vehicle_body: VehicleBody):
    """
    state: array with size (3,) for x, y, psi
    steer: float for steering angle
    """
    l = vehicle_body.l
    w = vehicle_body.w

    lf = vehicle_body.lf
    lr = vehicle_body.lr

    w_d = vehicle_body.wheel_d
    w_w = vehicle_body.wheel_w

    x, y, th = state

    # Body
    p = np.array([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]]) @ np.array([[l/2,l/2, -l/2, -l/2, l/2],[w/2, -w/2, -w/2, w/2, w/2]])

    ax.plot(p[0,:] + x, p[1,:] + y, 'k', linewidth = 1)

    # Rear wheels
    p = np.array([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]]) @ np.array([[w_d/2, w_d/2, -w_d/2, -w_d/2, w_d/2],[w_w/2, -w_w/2, -w_w/2, w_w/2, w_w/2]])

    ax.plot(p[0,:] + x - lr*np.cos(th) - (w-w_w)/2*np.sin(th), p[1,:] + y - lr*np.sin(th) + (w-w_w)/2*np.cos(th), 'k', linewidth = 0.5)

    ax.plot(p[0,:] + x - lr*np.cos(th) + (w-w_w)/2*np.sin(th), p[1,:] + y - lr*np.sin(th) - (w-w_w)/2*np.cos(th), 'k', linewidth = 0.5)

    # Front wheels
    p = np.array([[np.cos(th+steer), -np.sin(th+steer)],[np.sin(th+steer), np.cos(th+steer)]]) @ np.array([[w_d/2, w_d/2, -w_d/2, -w_d/2, w_d/2],[w_w/2, -w_w/2, -w_w/2, w_w/2, w_w/2]])

    ax.plot(p[0,:] + x + lf*np.cos(th) - (w-w_w)/2*np.sin(th), p[1,:] + y + lf*np.sin(th) + (w-w_w)/2*np.cos(th), 'k', linewidth = 0.5)

    ax.plot(p[0,:] + x + lf*np.cos(th) + (w-w_w)/2*np.sin(th), p[1,:] + y + lf*np.sin(th) - (w-w_w)/2*np.cos(th), 'k', linewidth = 0.5)

class OfflineVisualizer(object):
    """

    OfflineVisualizer provides tools for offline visualization of vehicle simulation results.
    Attributes:
        sol (VehiclePrediction): The predicted vehicle trajectory and control inputs.
        obstacles (list): List of obstacle objects to be visualized.
        map: The map object for the environment (type unspecified).
        vehicle_body (VehicleBody): The vehicle body model for rendering.
        region (GeofenceRegion): The region defining the plotting boundaries.
    Methods:
        plot_frame(ax, k):
            Plots the vehicle and obstacles at a specific time step k on the given matplotlib axis.
        plot_trace(ax):
            Plots the time series of vehicle speed, heading angle, acceleration, and steering input.
        plot_solution(step=1, fig_path=None, show=True):
            Plots the entire solution trajectory and vehicle states in a single figure.
            Optionally saves the figure and controls display.
        animate_solution(interval=40, gif_path=None, show=True):
            Animates the vehicle trajectory and state evolution over time.
            Optionally saves the animation as a GIF and controls display.
        show():
            Displays all open matplotlib figures.

    Visualize the results offline
    """
    def __init__(self, sol: VehiclePrediction, obstacles, map, vehicle_body: VehicleBody, region: GeofenceRegion):
        self.sol = sol
        self.obstacles = obstacles
        self.map = map
        self.vehicle_body = vehicle_body
        self.region = region

    def plot_frame(self, ax, k):
        xi = self.sol.x[k]
        xj = self.sol.y[k]
        th = self.sol.psi[k]
        steer = self.sol.u_steer[k]

        plot_vehicle(ax, [xi, xj, th], steer, self.vehicle_body)

        # Draw obstacles
        for obstacle in self.obstacles:
            obstacle.plot_pyplot(ax)

        ax.set_xlim([self.region.x_min, self.region.x_max])
        ax.set_ylim([self.region.y_min, self.region.y_max])
        ax.set_aspect('equal')

    def plot_trace(self, ax):
        ax.plot(self.sol.t, self.sol.v)
        ax.plot(self.sol.t, self.sol.psi)
        ax.plot(self.sol.t, self.sol.u_a)
        ax.plot(self.sol.t, self.sol.u_steer, ':')
        ax.legend(('speed','angle','accel','steer'))
        ax.set_xlabel('time')

    def plot_solution(self, step=1, fig_path=None, show=True):
        """
        Plot solution in one figure
        step: the stride step to plot vehicle contour. 1 is every step
        """
        plt.figure()

        ax1 = plt.subplot(2,1,1)

        # Plot the entire trajectory
        ax1.plot(self.sol.x, self.sol.y)
        ax1.set_xlabel('x')
        ax1.set_ylabel('y')

        for k in range(0, len(self.sol.t), step):
            self.plot_frame(ax1, k)

        ax2 = plt.subplot(2,1,2)
        self.plot_trace(ax2)

        if fig_path:
            plt.savefig(fig_path)

        if show:
            plt.show(block=False)

    def animate_solution(self, interval=40, gif_path=None, show=True):
        """
        animate the solution with the specified frame interval
        interval: frame interval in millesecond
        gif_path: the path to save gif, e.g. "maneuver.gif"
        """
        fig, ax = plt.subplots()

        def animate(k):
            ax.clear()
            # Plot the entire trajectory
            ax.plot(self.sol.x, self.sol.y)
            ax.set_xlabel('x')
            ax.set_ylabel('y')

            self.plot_frame(ax, k)

        ani = FuncAnimation(fig, animate, frames=len(self.sol.t), interval=interval, repeat=True)

        if gif_path:
            writer = PillowWriter(fps=int(1000/interval))
            ani.save(gif_path, writer=writer)

        if show:
            plt.show()

        return

    def show(self):
        plt.show()
        return
