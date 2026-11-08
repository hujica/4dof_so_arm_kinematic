#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class SOArm101GUI:
    def __init__(self, root_window: tk.Tk, controller, ros_node):
        self.root = root_window
        self.root.title("SO-ARM101 Controller & 3D Simulation Mode")
        self.root.geometry("1000x650")
        
        self.arm_controller = controller
        self.ros_node = ros_node
        self.init_gui()
        self.update_visualizer_3d()

    def init_gui(self):
        ttk.Label(self.root, text="SO-ARM101 Mission Controller (Simulation Mode)", font=('Helvetica', 14, 'bold')).pack(pady=10)
        
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Left Control Panel
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side="left", fill="y", padx=10)

        # Robot Pose Display
        pose_frame = ttk.LabelFrame(left_panel, text="Robot Pose (Cartesian)")
        pose_frame.pack(fill="x", pady=5)
        
        ttk.Label(pose_frame, text="X (m)").grid(row=0, column=0, padx=15)
        ttk.Label(pose_frame, text="Y (m)").grid(row=0, column=1, padx=15)
        ttk.Label(pose_frame, text="Z (m)").grid(row=0, column=2, padx=15)
        
        self.lbl_x = ttk.Label(pose_frame, text="0.00", font=('Helvetica', 11, 'bold'))
        self.lbl_x.grid(row=1, column=0, pady=5)
        self.lbl_y = ttk.Label(pose_frame, text="0.00", font=('Helvetica', 11, 'bold'))
        self.lbl_y.grid(row=1, column=1, pady=5)
        self.lbl_z = ttk.Label(pose_frame, text="0.00", font=('Helvetica', 11, 'bold'))
        self.lbl_z.grid(row=1, column=2, pady=5)

        # Cartesian Jog Controls
        ctrl_frame = ttk.LabelFrame(left_panel, text="Cartesian Jog Control (+/- 2cm)")
        ctrl_frame.pack(fill="x", pady=10)
        
        ttk.Button(ctrl_frame, text="+X (Fwd)", command=lambda: self.on_cartesian_jog("x", 0.02)).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(ctrl_frame, text="-X (Back)", command=lambda: self.on_cartesian_jog("x", -0.02)).grid(row=2, column=1, padx=2, pady=2)
        
        ttk.Button(ctrl_frame, text="+Y (Right)", command=lambda: self.on_cartesian_jog("y", 0.02)).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(ctrl_frame, text="-Y (Left)", command=lambda: self.on_cartesian_jog("y", -0.02)).grid(row=1, column=0, padx=2, pady=2)
        
        ttk.Button(ctrl_frame, text="+Z (Up)", command=lambda: self.on_cartesian_jog("z", 0.02)).grid(row=0, column=2, padx=2, pady=2)
        ttk.Button(ctrl_frame, text="-Z (Down)", command=lambda: self.on_cartesian_jog("z", -0.02)).grid(row=2, column=2, padx=2, pady=2)
        
        ttk.Button(ctrl_frame, text="Home", command=self.on_home_arm).grid(row=1, column=1, padx=2, pady=2)

        # Status Line Label
        self.lbl_status = ttk.Label(left_panel, text="Status: Ready", font=('Helvetica', 10, 'italic'), foreground="green")
        self.lbl_status.pack(pady=5)

        # E-Stop Button
        tk.Button(left_panel, text="EMERGENCY STOP", bg="red", fg="white", font=('Helvetica', 11, 'bold'),
                  command=self.arm_controller.emergency_stop).pack(fill="x", pady=10)

        # Right 3D Visualizer Panel
        right_panel = ttk.LabelFrame(main_frame, text="3D Arm Kinematic Simulation")
        right_panel.pack(side="right", fill="both", expand=True, padx=10)

        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def compute_joint_positions(self):
        l1, l2, l3, l4 = self.arm_controller.kinematics.link_lengths
        q_rad = np.radians(self.arm_controller.current_joint_angles)
        th1, th2, th3, th4 = q_rad[0], q_rad[1], q_rad[2], q_rad[3]

        p0 = [0, 0, 0]
        p1 = [0, 0, l1]
        r2 = l2 * np.cos(th2)
        p2 = [r2 * np.cos(th1), r2 * np.sin(th1), l1 + l2 * np.sin(th2)]
        r3 = r2 + l3 * np.cos(th2 + th3)
        p3 = [r3 * np.cos(th1), r3 * np.sin(th1), p2[2] + l3 * np.sin(th2 + th3)]
        r4 = r3 + l4 * np.cos(th2 + th3 + th4)
        p4 = [r4 * np.cos(th1), r4 * np.sin(th1), p3[2] + l4 * np.sin(th2 + th3 + th4)]

        pts = np.array([p0, p1, p2, p3, p4])
        return pts[:, 0], pts[:, 1], pts[:, 2]

    def update_visualizer_3d(self):
        pose = self.arm_controller.current_end_effector_pose
        self.lbl_x.config(text=f"{pose[0]:.2f}")
        self.lbl_y.config(text=f"{pose[1]:.2f}")
        self.lbl_z.config(text=f"{pose[2]:.2f}")

        self.ax.clear()
        xs, ys, zs = self.compute_joint_positions()

        self.ax.plot(xs, ys, zs, '-o', linewidth=4, markersize=8, color='blue', label='SO-ARM101 Structure')
        self.ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color='red', s=60, label='End Effector')

        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0.0, 0.4])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.legend(loc='upper right')
        self.canvas.draw()

    def on_home_arm(self):
        self.arm_controller.home_arm()
        self.update_visualizer_3d()
        self.lbl_status.config(text="Status: Reset to Home Pose", foreground="green")

    def on_cartesian_jog(self, axis: str, step: float):
        dx = step if axis == 'x' else 0.0
        dy = step if axis == 'y' else 0.0
        dz = step if axis == 'z' else 0.0

        if self.arm_controller.move_cartesian(dx, dy, dz):
            self.update_visualizer_3d()
            self.lbl_status.config(text=f"Status: Moved {axis.upper()} ({step:+.2f}m)", foreground="green")
            if self.ros_node:
                self.ros_node.publish_joint_states(self.arm_controller.current_joint_angles)
        else:
            self.lbl_status.config(text=f"Status: {axis.upper()} Move Blocked (Limit Reached)", foreground="red")