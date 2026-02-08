import numpy as np
import matplotlib.pyplot as plt
from typing import Any
from constants import Constants


class Visualizer:
    def __init__(self, model):
        self.hub_b = np.zeros(3, dtype=float)
        self.arms_b = []
        self.rotors_b = []

        self.fig = None
        self.ax = None

        self.arm_lines = []
        self.rotor_lines = []
        self.hub_scatter = None

        self.world_triad_lines = None
        self.world_triad_texts = None

        self.body_triad_lines = None
        self.body_triad_texts = None

        self.body_triad_scale = 0.18
        self.world_triad_scale = 0.25
        self.room_scale = 2
        self.model = model
        self.set_up(self.model.mc.x0[0:3],self.model.mc.x0[6:10])
        
    def make_octa_geometry(self, L, Rr, n_circle, z_rotor):
        hub = np.array([0.0, 0.0, 0.0], dtype=float)

        arms = []
        rotors = []

        for k in range(8):
            theta = k * (np.pi / 4.0)
            rotor_center = np.array([L * np.cos(theta), L * np.sin(theta), z_rotor], dtype=float)

            arms.append(np.vstack([hub, rotor_center]))

            phi = np.linspace(0, 2 * np.pi, n_circle, endpoint=False)
            ring = np.column_stack([
                    rotor_center[0] + Rr * np.cos(phi),
                    rotor_center[1] + Rr * np.sin(phi),
                    np.full(phi.shape, rotor_center[2], dtype=float),
                    ])
            rotors.append(ring)

        return hub, arms, rotors


    def combine_points(self, hub, arms, rotors):
        pts = [hub.reshape(1, 3)]
        pts += arms
        pts += rotors
        return np.vstack(pts)


    def set_axes_equal_from_points(self, ax, pts, room_scale):
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        centers = 0.5 * (mins + maxs)
        ranges = maxs - mins
        plot_radius = 0.5 * np.max(ranges) * room_scale

        ax.set_xlim(centers[0] - plot_radius, centers[0] + plot_radius)
        ax.set_ylim(centers[1] - plot_radius, centers[1] + plot_radius)
        ax.set_zlim(centers[2] - plot_radius, centers[2] + plot_radius)


    def quat_normalize_xyzw(self, q):
        q = np.asarray(q, dtype=float).reshape(4,)
        n = np.linalg.norm(q)
        if n <= 0.0:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return q / n


    def compute_body_to_world(self, q):
        R = np.array([
                [1.0 - 2.0 * (q[1]**2 + q[2]**2), 2.0 * (q[0]*q[1] - q[2]*q[3]),       2.0 * (q[0]*q[2] + q[1]*q[3])],
                [2.0 * (q[0]*q[1] + q[2]*q[3]),       1.0 - 2.0 * (q[0]**2 + q[2]**2), 2.0 * (q[1]*q[2] - q[0]*q[3])],
                [2.0 * (q[0]*q[2] - q[1]*q[3]),       2.0 * (q[1]*q[2] + q[0]*q[3]),       1.0 - 2.0 * (q[0]**2 + q[1]**2)],
            ], dtype=float,)
        return R


    def apply_rotation(self, R, vector):
        return (R @ vector.T).T


    def apply_translation(self, offset, point):
        offset = np.asarray(offset, dtype=float).reshape(1, 3)
        return point + offset


    def plot_world_axes(self, ax, origin, scale, lw):
        origin = np.asarray(origin, dtype=float).reshape(3,)
        R_cols = np.eye(3)

        end_x = origin + scale * R_cols[:, 0]
        end_y = origin + scale * R_cols[:, 1]
        end_z = origin + scale * R_cols[:, 2]

        ax.plot([origin[0], end_x[0]], [origin[1], end_x[1]], [origin[2], end_x[2]], linewidth=lw)
        ax.plot([origin[0], end_y[0]], [origin[1], end_y[1]], [origin[2], end_y[2]], linewidth=lw)
        ax.plot([origin[0], end_z[0]], [origin[1], end_z[1]], [origin[2], end_z[2]], linewidth=lw)

        ax.text(end_x[0], end_x[1], end_x[2], "$x_w$", fontsize=10)
        ax.text(end_y[0], end_y[1], end_y[2], "$y_w$", fontsize=10)
        ax.text(end_z[0], end_z[1], end_z[2], "$z_w$", fontsize=10)

    def build_arm_lines(self):
        self.arm_lines = []
        for i in range(8):
            ln = self.ax.plot([], [], [], color="black", linewidth=2)[0]
            self.arm_lines.append(ln)

    def build_rotor_lines(self):
        self.rotor_lines = []
        for _ in range(8):
            line = self.ax.plot([], [], [], color="blue", linewidth=1.0)[0]
            self.rotor_lines.append(line)

    def build_body_triad(self):
        bx = self.ax.plot([], [], [], linewidth=2.8)[0]
        by = self.ax.plot([], [], [], linewidth=2.8)[0]
        bz = self.ax.plot([], [], [], linewidth=2.8)[0]
        self.body_triad_lines = (bx, by, bz)

    def build_world_triad(self):
        tx = self.ax.text(0, 0, 0, r"$x_b$", fontsize=10)
        ty = self.ax.text(0, 0, 0, r"$y_b$", fontsize=10)
        tz = self.ax.text(0, 0, 0, r"$z_b$", fontsize=10)
        self.body_triad_texts = (tx, ty, tz)

    def set_up(self, r_init, q_init):
        self.hub_b, self.arms_b, self.rotors_b = self.make_octa_geometry(
            self.model.mc.L, self.model.mc.prop_rad, n_circle=40, z_rotor=0.03
        )

        plt.ion()
        self.fig = plt.figure()
        self.ax: Any = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Octacopter")

        self.ax.set_xlabel(r"$x_{world}$")
        self.ax.set_ylabel(r"$y_{world}$")
        self.ax.set_zlabel(r"$z_{world}$")


        self.build_arm_lines()
        self.build_rotor_lines()

        self.hub_scatter = self.ax.scatter([0.0], [0.0], [0.0], s=40)

        self.plot_world_axes(
            self.ax,
            origin=np.array([0.0, 0.0, 0.0]),
            scale=self.world_triad_scale,
            lw=2.0,
        )
        self.build_body_triad()
        self.build_world_triad()

        self.update(r_init, q_init)

        plt.show()

    def update(self, r, q):
        if self.fig is None or self.ax is None or self.hub_scatter is None:
            raise RuntimeError("Call set_up() before update().")
        if self.body_triad_lines is None or self.body_triad_texts is None:
            raise RuntimeError("Call set_up() before update().")

        bx, by, bz = self.body_triad_lines
        tx, ty, tz = self.body_triad_texts

        r_world = np.asarray(r, dtype=float).reshape(3,)
        q_xyzw = np.asarray(q, dtype=float).reshape(4,)
        R_bw = self.compute_body_to_world(q_xyzw)

        hub_w = r_world

        for i, seg_b in enumerate(self.arms_b):
            seg_w = self.apply_rotation(R_bw, seg_b) + r_world.reshape(1, 3)
            self.arm_lines[i].set_data(seg_w[:, 0], seg_w[:, 1])
            self.arm_lines[i].set_3d_properties(seg_w[:, 2])

        for i, ring_b in enumerate(self.rotors_b):
            ring_w = self.apply_rotation(R_bw, ring_b) + r_world.reshape(1, 3)
            self.rotor_lines[i].set_data(ring_w[:, 0], ring_w[:, 1])
            self.rotor_lines[i].set_3d_properties(ring_w[:, 2])

        self.hub_scatter._offsets3d = ([hub_w[0]], [hub_w[1]], [hub_w[2]])

        origin = hub_w
        scale = self.body_triad_scale
        ex = origin + scale * R_bw[:, 0]
        ey = origin + scale * R_bw[:, 1]
        ez = origin + scale * R_bw[:, 2]

        bx, by, bz = self.body_triad_lines
        bx.set_data([origin[0], ex[0]], [origin[1], ex[1]]); bx.set_3d_properties([origin[2], ex[2]])
        by.set_data([origin[0], ey[0]], [origin[1], ey[1]]); by.set_3d_properties([origin[2], ey[2]])
        bz.set_data([origin[0], ez[0]], [origin[1], ez[1]]); bz.set_3d_properties([origin[2], ez[2]])

        tx, ty, tz = self.body_triad_texts
        tx.set_position((ex[0], ex[1])); tx.set_3d_properties(ex[2])
        ty.set_position((ey[0], ey[1])); ty.set_3d_properties(ey[2])
        tz.set_position((ez[0], ez[1])); tz.set_3d_properties(ez[2])

        pts_geom = self.combine_points(
            hub_w,
            [self.apply_rotation(R_bw, seg) + r_world for seg in self.arms_b],
            [self.apply_rotation(R_bw, ring) + r_world for ring in self.rotors_b],
        )
        pts_all = np.vstack([pts_geom, ex.reshape(1, 3), ey.reshape(1, 3), ez.reshape(1, 3)])
        self.set_axes_equal_from_points(self.ax, pts_all, room_scale=self.room_scale)

        self.fig.canvas.draw_idle()
        plt.pause(self.model.mc.dt)
