#!/usr/bin/env python3
import math
import signal
import time
import traceback
from collections import deque
import tkinter as tk
from tkinter import ttk
import numpy as np

try:
    import rclpy
except ImportError:
    rclpy = None

C = dict(
    bg="#0d1117", panel="#161b22", card="#1c2330", btn="#263248", edge="#2b3445",
    text="#e6edf3", muted="#8b98a9", accent="#4fc3f7", violet="#9d7cff",
    ok="#3ddc97", warn="#ffb74d", danger="#ff5370", grid="#1f2736"
)
JOINT_COLORS = ["#4fc3f7", "#9d7cff", "#3ddc97", "#ffb74d"]
JOINT_NAMES = ["J1  Base", "J2  Shoulder", "J3  Elbow", "J4  Wrist"]
MONO = "Consolas"
LIMIT = 60


class SOArm101GUI:
    TICK_MS = 20          # control loop: 50 Hz
    DRAW_EVERY = 2        # redraw canvas at ~25 Hz

    def __init__(self, root_window: tk.Tk, controller, ros_node):
        self.root = root_window
        self.root.title("SO-ARM101  |  Velocity Kinematics Console")
        #self.root.geometry("1320x820")
        try:
            self.root.state("zoomed")
        except tk.TclError:
            try:
                self.root.attributes("-zoomed", True)
            except tk.TclError:
                w, h = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
                self.root.geometry(f"{w}x{h}+0+0")
        self.root.minsize(800, 480)

        self.arm_controller = controller
        self.ctrl = controller
        self.ros_node = ros_node

        self.jog_dir = None
        self.dragging = set()
        self.syncing = False
        self._closing = False
        self.history = deque(maxlen=500)      # 10 s of joint velocities @ 50 Hz
        self.trail = deque(maxlen=200)
        self.t0 = self.last = time.perf_counter()
        self.n = 0
        self.last_err = None
        self._last_status = None

        # 3D camera orbit angles (degrees)
        self.cam_azim = -55.0
        self.cam_elev = 24.0
        self._drag_xy = None

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        # Esc = emergency stop (stop only; reset must be done with the button)
        self.root.bind("<Escape>", lambda e: None if self.ctrl.estopped else self._toggle_estop())
        try:
            signal.signal(signal.SIGINT, lambda sig, frame: self.on_close())
        except Exception:
            pass

        self._build_style()
        self._build_layout()
        self.tick()

    # ================================================================ styling
    def _build_style(self):
        self.root.configure(bg=C["bg"])
        s = ttk.Style(self.root)
        s.theme_use("clam")
        s.configure(".", background=C["bg"], foreground=C["text"], font=("Segoe UI", 10))
        s.configure("Card.TFrame", background=C["card"], bordercolor=C["edge"],
                    borderwidth=1, relief="solid")
        s.configure("Inner.TFrame", background=C["card"])
        s.configure("Card.TLabel", background=C["card"], foreground=C["text"])
        s.configure("Title.TLabel", background=C["card"], foreground=C["accent"],
                    font=("Segoe UI", 9, "bold"))
        s.configure("Muted.TLabel", background=C["card"], foreground=C["muted"], font=("Segoe UI", 9))
        s.configure("Head.TLabel", font=("Segoe UI", 20, "bold"))
        s.configure("Sub.TLabel", foreground=C["muted"], font=("Segoe UI", 10))
        s.configure("Jog.TButton", background=C["btn"], foreground=C["text"], bordercolor=C["edge"],
                    focusthickness=0, padding=(6, 9), font=("Segoe UI", 10, "bold"))
        s.map("Jog.TButton",
              background=[("pressed", C["accent"]), ("active", "#324260")],
              foreground=[("pressed", C["bg"])])
        s.configure("Home.TButton", background=C["violet"], foreground="white", bordercolor=C["violet"],
                    focusthickness=0, padding=(6, 9), font=("Segoe UI", 10, "bold"))
        s.map("Home.TButton", background=[("pressed", "#7d5fe0"), ("active", "#b199ff")])
        s.configure("Horizontal.TScale", background=C["card"], troughcolor=C["bg"],
                    bordercolor=C["edge"], lightcolor=C["accent"], darkcolor=C["accent"])
        s.configure("TSpinbox", fieldbackground=C["bg"], foreground=C["text"], background=C["btn"],
                    arrowcolor=C["text"], bordercolor=C["edge"], insertcolor=C["text"])

    def _card(self, parent, title):
        outer = ttk.Frame(parent, style="Card.TFrame", padding=(14, 10))
        outer.pack(fill="x", pady=5)
        ttk.Label(outer, text=title.upper(), style="Title.TLabel").pack(anchor="w", pady=(0, 8))
        body = ttk.Frame(outer, style="Inner.TFrame")
        body.pack(fill="x")
        return body

    def _mono(self, parent, text="", size=11, fg=None, width=None, anchor="w"):
        return tk.Label(parent, text=text, bg=C["card"], fg=fg or C["text"],
                        font=(MONO, size, "bold"), width=width, anchor=anchor)

    # ================================================================= layout
    def _build_layout(self):
        head = ttk.Frame(self.root)
        head.pack(fill="x", padx=20, pady=(14, 4))
        ttk.Label(head, text="SO-ARM101", style="Head.TLabel").pack(side="left")
        ttk.Label(head, text="   4-DOF  ·  Velocity Kinematics Console", style="Sub.TLabel"
                  ).pack(side="left", pady=(8, 0))
        self.chip = tk.Label(head, text="READY", bg=C["ok"], fg=C["bg"], font=("Segoe UI", 10, "bold"),
                             padx=14, pady=4)
        self.chip.pack(side="right")
        self.lbl_mode = ttk.Label(head, text="IDLE", style="Sub.TLabel")
        self.lbl_mode.pack(side="right", padx=16)
        hw = self.ctrl.hardware.is_connected
        tk.Label(head, text="● HARDWARE LIVE" if hw else "● SIMULATION", bg=C["bg"],
                 fg=C["ok"] if hw else C["warn"], font=("Segoe UI", 9, "bold")).pack(side="right", padx=8)

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=20, pady=8)
        left = ttk.Frame(body, width=430)
        left.pack(side="left", fill="y", padx=(0, 16))
        right = ttk.Frame(body)
        right.pack(side="right", fill="both", expand=True)

        self._build_pose_card(left)
        self._build_jog_card(left)
        self._build_speed_card(left)
        self._build_joint_card(left)

        """self.btn_estop = tk.Button(left, text="⏻   EMERGENCY STOP", bg=C["danger"], fg="white",
                                   activebackground="#ff7a90", activeforeground="white",
                                   font=("Segoe UI", 12, "bold"), relief="flat", bd=0, pady=11,
                                   cursor="hand2", command=self._toggle_estop)
        self.btn_estop.pack(fill="x", pady=(10, 0))"""

        self._build_viewports(right)

    def _build_pose_card(self, parent):
        b = self._card(parent, "End-effector pose & velocity")
        self.pose_lbls = []
        for col, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("PITCH", "°")]):
            ttk.Label(b, text=f"{name} ({unit})", style="Muted.TLabel").grid(row=0, column=col, padx=8, sticky="w")
            lbl = self._mono(b, "0.0", 16, C["accent"], width=7)
            lbl.grid(row=1, column=col, padx=4, pady=(0, 2))
            self.pose_lbls.append(lbl)
            b.columnconfigure(col, weight=1)
        ttk.Label(b, text="TIP VELOCITY", style="Muted.TLabel").grid(row=2, column=0, columnspan=4,
                                                                     padx=2, sticky="w")
        self.lbl_tip_v = self._mono(b, "", 10, C["ok"])
        self.lbl_tip_v.grid(row=3, column=0, columnspan=4, padx=8, sticky="w")

    def _build_jog_card(self, parent):
        b = self._card(parent, "Cartesian jog  ·  hold to move")
        pad = [("▲  +X", (1, 0, 0), 0, 1), ("▼  −X", (-1, 0, 0), 2, 1),
               ("◀  −Y", (0, 1, 0), 1, 0), ("+Y  ▶", (0, -1, 0), 1, 2),
               ("⬆  +Z", (0, 0, 1), 0, 4), ("⬇  −Z", (0, 0, -1), 2, 4)]
        for text, vec, r, c in pad:
            btn = ttk.Button(b, text=text, style="Jog.TButton", width=8)
            btn.grid(row=r, column=c, padx=3, pady=2, sticky="nsew")
            btn.bind("<ButtonPress-1>", lambda e, v=vec: setattr(self, "jog_dir", v))
            btn.bind("<ButtonRelease-1>", lambda e: setattr(self, "jog_dir", None))
        ttk.Button(b, text="HOME", style="Home.TButton", width=8, command=self._on_home_arm
                   ).grid(row=1, column=1, padx=3, pady=2, sticky="nsew")
        b.columnconfigure(3, minsize=18)
        ttk.Label(b, text="X forward · Y right · Z up   (resolved-rate control)", style="Muted.TLabel"
                  ).grid(row=3, column=0, columnspan=5, pady=(2, 0), sticky="w")

    def _build_speed_card(self, parent):
        b = self._card(parent, "Velocity input")
        self.lin_var = tk.DoubleVar(value=40)
        self.ang_var = tk.DoubleVar(value=30)
        rows = [("Linear (tip)", self.lin_var, 5, 150, 5, "mm/s"),
                ("Joint max", self.ang_var, 5, 120, 5, "°/s")]
        for r, (name, var, lo, hi, inc, unit) in enumerate(rows):
            ttk.Label(b, text=name, style="Card.TLabel").grid(row=r, column=0, sticky="w", pady=2)
            ttk.Scale(b, from_=lo, to=hi, variable=var, length=170,
                      command=lambda v, var=var: var.set(round(float(v)))
                      ).grid(row=r, column=1, padx=10)
            ttk.Spinbox(b, from_=lo, to=hi, increment=inc, textvariable=var, width=5
                        ).grid(row=r, column=2)
            ttk.Label(b, text=unit, style="Muted.TLabel").grid(row=r, column=3, padx=(2, 0))

    def _build_joint_card(self, parent):
        b = self._card(parent, f"Joint targets  ·  −{LIMIT}° … +{LIMIT}°")
        for col, t in ((2, "ACTUAL"), (3, "VELOCITY")):
            ttk.Label(b, text=t, style="Muted.TLabel").grid(row=0, column=col, sticky="w", padx=4)
        self.sliders, self.lbl_act, self.lbl_vel = [], [], []
        q0 = self.ctrl.current_joint_angles
        for i in range(4):
            r = i + 1
            tk.Label(b, text=JOINT_NAMES[i], bg=C["card"], fg=JOINT_COLORS[i],
                     font=("Segoe UI", 10, "bold"), width=11, anchor="w").grid(row=r, column=0, pady=3)
            sc = ttk.Scale(b, from_=-LIMIT, to=LIMIT, length=130,
                           command=lambda v, i=i: self._on_slider(i, v))
            sc.set(q0[i])
            sc.grid(row=r, column=1, padx=6)
            sc.bind("<ButtonPress-1>", lambda e, i=i: self.dragging.add(i))
            sc.bind("<ButtonRelease-1>", lambda e, i=i: self.dragging.discard(i))
            la = self._mono(b, "", 10, width=8)
            la.grid(row=r, column=2, padx=4)
            lv = self._mono(b, "", 10, C["muted"], width=10)
            lv.grid(row=r, column=3, padx=4)
            self.sliders.append(sc)
            self.lbl_act.append(la)
            self.lbl_vel.append(lv)

    def _build_viewports(self, parent):
        # Top: 3D Kinematic Canvas (Pure Tkinter)
        arm_frame = ttk.Frame(parent, style="Card.TFrame", padding=(10, 8))
        arm_frame.pack(fill="both", expand=True, pady=(5, 6))

        hdr = ttk.Frame(arm_frame, style="Inner.TFrame")
        hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(hdr, text="3D KINEMATIC VISUALIZER", style="Title.TLabel").pack(side="left")
        ttk.Label(hdr, text="Drag canvas to rotate view", style="Muted.TLabel").pack(side="right")

        self.arm_canvas = tk.Canvas(arm_frame, bg=C["bg"], highlightthickness=0, height=360)
        self.arm_canvas.pack(fill="both", expand=True)
        self.arm_canvas.bind("<ButtonPress-1>", self._on_cam_press)
        self.arm_canvas.bind("<B1-Motion>", self._on_cam_drag)

        # Bottom: Joint Velocity Plot Canvas (Pure Tkinter)
        vel_frame = ttk.Frame(parent, style="Card.TFrame", padding=(10, 8))
        vel_frame.pack(fill="x", pady=(0, 0))

        vhdr = ttk.Frame(vel_frame, style="Inner.TFrame")
        vhdr.pack(fill="x", pady=(0, 4))
        ttk.Label(vhdr, text="JOINT VELOCITY HISTORY (°/s  ·  10s WINDOW)", style="Title.TLabel").pack(side="left")
        leg = ttk.Frame(vhdr, style="Inner.TFrame")
        leg.pack(side="right")
        for i in range(4):
            tk.Label(leg, text=f"■ J{i+1}", bg=C["card"], fg=JOINT_COLORS[i],
                     font=("Segoe UI", 9, "bold")).pack(side="left", padx=6)

        self.vel_canvas = tk.Canvas(vel_frame, bg=C["bg"], highlightthickness=0, height=135)
        self.vel_canvas.pack(fill="x")

        # Emergency Stop Button (bottom of left panel)
        self.btn_estop = tk.Button(parent, text="⏻   EMERGENCY STOP", bg=C["danger"], fg="white",
                                           activebackground="#ff7a90", activeforeground="white",
                                           font=("Segoe UI", 12, "bold"), relief="flat", bd=0, pady=6,
                                           cursor="hand2", command=self._toggle_estop)
        self.btn_estop.pack(fill="x", pady=(6, 0))

    # ======================================================= 3D math & canvas
    def _on_cam_press(self, event):
        self._drag_xy = (event.x, event.y)

    def _on_cam_drag(self, event):
        if self._drag_xy is None:
            return
        dx = event.x - self._drag_xy[0]
        dy = event.y - self._drag_xy[1]
        self._drag_xy = (event.x, event.y)
        self.cam_azim = (self.cam_azim - dx * 0.5) % 360
        self.cam_elev = float(np.clip(self.cam_elev + dy * 0.4, -15.0, 85.0))

    def _project_3d(self, pts, w, h):
        """Orthographic 3D -> 2D screen projection centered on workspace."""
        az = math.radians(self.cam_azim)
        el = math.radians(self.cam_elev)
        ca, sa = math.cos(az), math.sin(az)
        ce, se = math.cos(el), math.sin(el)

        # Center workspace around (x=0.10, y=0.0, z=0.16)
        p = np.asarray(pts, dtype=float) - np.array([0.10, 0.0, 0.16])
        x, y, z = p[:, 0], p[:, 1], p[:, 2]

        # Rotate around Z (azimuth), then X (elevation)
        xr = -x * sa + y * ca
        yr = x * ca + y * sa
        zr = z * ce - yr * se

        scale = min(w, h) * 1.45
        sx = w * 0.5 + xr * scale
        sy = h * 0.54 - zr * scale
        return np.column_stack((sx, sy))

    def _compute_joint_positions(self):
        # Take the points from the kinematics so the DH shoulder offset (a1 = 4.2 cm) is drawn too
        return np.asarray(self.ctrl.kinematics.joint_positions(self.ctrl.current_joint_angles), dtype=float)

    def _draw_3d_scene(self):
        cv = self.arm_canvas
        w = max(cv.winfo_width(), 300)
        h = max(cv.winfo_height(), 250)
        cv.delete("all")

        # Ground floor grid (Z = 0)
        grid_vals = np.linspace(-0.25, 0.35, 7)
        for g in grid_vals:
            seg_x = self._project_3d([[g, -0.25, 0.0], [g, 0.25, 0.0]], w, h)
            cv.create_line(*seg_x[0], *seg_x[1], fill=C["grid"], width=1)
        for g in np.linspace(-0.25, 0.25, 6):
            seg_y = self._project_3d([[-0.25, g, 0.0], [0.35, g, 0.0]], w, h)
            cv.create_line(*seg_y[0], *seg_y[1], fill=C["grid"], width=1)

        # Coordinate axes at origin (X=red/danger, Y=green/ok, Z=blue/accent)
        axes = [
            ([0.08, 0, 0], C["danger"], "X"),
            ([0, 0.08, 0], C["ok"], "Y"),
            ([0, 0, 0.08], C["accent"], "Z"),
        ]
        orig_2d = self._project_3d([[0, 0, 0]], w, h)[0]
        for vec, col, label in axes:
            tip_2d = self._project_3d([vec], w, h)[0]
            cv.create_line(*orig_2d, *tip_2d, fill=col, width=2)
            cv.create_text(tip_2d[0] + 6, tip_2d[1], text=label, fill=col, font=(MONO, 9, "bold"))

        # Base ring
        th = np.linspace(0, 2 * np.pi, 36)
        ring = np.column_stack((0.045 * np.cos(th), 0.045 * np.sin(th), np.zeros_like(th)))
        ring_2d = self._project_3d(ring, w, h)
        cv.create_polygon(*ring_2d.flatten(), outline=C["muted"], fill="", width=2)

        pts_3d = self._compute_joint_positions()
        pts_2d = self._project_3d(pts_3d, w, h)

        # Ground shadow
        shadow_3d = pts_3d.copy()
        shadow_3d[:, 2] = 0.0
        shadow_2d = self._project_3d(shadow_3d, w, h)
        cv.create_line(*shadow_2d.flatten(), fill="#222b3a", width=5, capstyle="round")

        # End-effector trail
        if len(self.trail) > 1:
            tr_2d = self._project_3d(np.array(self.trail), w, h)
            cv.create_line(*tr_2d.flatten(), fill=C["violet"], width=2)

        # Robot arm links
        for i in range(len(pts_2d) - 1):
            cv.create_line(*pts_2d[i], *pts_2d[i + 1], fill=C["accent"], width=8, capstyle="round")

        # Joints
        for i in range(len(pts_2d) - 1):
            x, y = pts_2d[i]
            cv.create_oval(x - 6, y - 6, x + 6, y + 6, fill="white", outline=C["accent"], width=2)

        # End-effector tip + velocity arrow
        tx, ty = pts_2d[-1]
        v = np.array(self.ctrl.current_tip_velocity)
        if np.linalg.norm(v) > 1e-4:
            end_3d = pts_3d[-1] + v * 2.0
            end_2d = self._project_3d([end_3d], w, h)[0]
            cv.create_line(tx, ty, *end_2d, fill=C["ok"], width=3, arrow="last")
        cv.create_oval(tx - 7, ty - 7, tx + 7, ty + 7, fill=C["danger"], outline="white", width=1.5)

    def _draw_velocity_plot(self, ang_limit):
        cv = self.vel_canvas
        w = max(cv.winfo_width(), 300)
        h = max(cv.winfo_height(), 140)
        cv.delete("all")

        pad_l, pad_r, pad_t, pad_b = 46, 14, 12, 22
        pw = max(w - pad_l - pad_r, 10)
        ph = max(h - pad_t - pad_b, 10)
        lim = max(ang_limit * 1.15, 10.0)

        # Horizontal grid + Y labels
        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
            val = frac * lim
            y = pad_t + ph * (0.5 - 0.5 * frac)
            col = C["edge"] if frac == 0.0 else C["grid"]
            cv.create_line(pad_l, y, pad_l + pw, y, fill=col, width=1)
            cv.create_text(pad_l - 6, y, text=f"{val:4.0f}", fill=C["muted"], anchor="e", font=(MONO, 8))

        # Vertical grid (-10s .. 0s)
        for sec in range(-10, 1, 2):
            x = pad_l + pw * ((sec + 10.0) / 10.0)
            cv.create_line(x, pad_t, x, pad_t + ph, fill=C["grid"], width=1)
            cv.create_text(x, pad_t + ph + 11, text=f"{sec}s", fill=C["muted"], font=(MONO, 8))

        if len(self.history) > 1:
            d = np.array(self.history)
            t = d[:, 0] - d[-1, 0]
            mask = t >= -10.0
            t = t[mask]
            d = d[mask]
            if len(t) > 1:
                xs = pad_l + pw * ((t + 10.0) / 10.0)
                for j in range(4):
                    ys = pad_t + ph * (0.5 - 0.5 * np.clip(d[:, j + 1] / lim, -1.0, 1.0))
                    coords = np.column_stack((xs, ys)).flatten().tolist()
                    cv.create_line(*coords, fill=JOINT_COLORS[j], width=2)

    # =============================================================== callbacks
    def _read(self, var, lo, hi, default):
        try:
            return float(np.clip(float(var.get()), lo, hi))
        except (tk.TclError, ValueError):
            return default

    def _on_slider(self, i, val):
        if self.syncing:
            return
        self.ctrl.set_joint_target(i, round(float(val), 1))

    def _on_home_arm(self):
        self.ctrl.home_arm()

    def _toggle_estop(self):
        if self.ctrl.estopped:
            self.ctrl.reset_estop()
            self.btn_estop.configure(text="⏻   EMERGENCY STOP", bg=C["danger"], fg="white")
        else:
            self.jog_dir = None
            self.ctrl.emergency_stop()
            self.btn_estop.configure(text="↺   RESET E-STOP", bg=C["warn"], fg=C["bg"])

    def on_close(self):
        if self._closing:
            return
        self._closing = True
        self.ros_node = None
        try:
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    # ================================================================ main loop
    def tick(self):
        if self._closing:
            return
        if rclpy is not None and self.ros_node is not None and not rclpy.ok():
            self.on_close()
            return

        try:
            now = time.perf_counter()
            dt = min(max(now - self.last, 1e-3), 0.1)
            self.last = now
            c = self.ctrl

            lin = self._read(self.lin_var, 5, 150, 40)
            ang = self._read(self.ang_var, 5, 120, 30)
            c.set_speed_limits(lin / 1000.0, ang)
            if self.jog_dir is not None and not c.estopped:
                c.set_cartesian_velocity(*(np.array(self.jog_dir, float) * c.max_linear_speed))
            else:
                c.set_cartesian_velocity(0.0, 0.0, 0.0)
            c.update(dt)

            # controller can trigger E-stop itself (e.g. servo comm lost) -> sync button
            if c.estopped and not self.btn_estop.cget("text").startswith("↺"):
                self.jog_dir = None
                self.btn_estop.configure(text="↺   RESET E-STOP", bg=C["warn"], fg=C["bg"])
            if c.status != self._last_status:
                if c.status_level != "ok":
                    print(f"[WARN] {c.status}")
                self._last_status = c.status

            self.history.append((now - self.t0, *c.current_joint_velocities))
            if np.linalg.norm(c.current_tip_velocity) > 1e-5:
                self.trail.append(c.current_end_effector_pose[:3])

            if self.ros_node and self.n % 2 == 0:
                self._publish_ros(c)

            if self.n % self.DRAW_EVERY == 0:
                self._refresh_readouts(ang)
                self._draw_3d_scene()
                self._draw_velocity_plot(ang)
            self.n += 1
        except Exception:
            msg = traceback.format_exc()
            if msg != self.last_err:
                print(msg)
                self.last_err = msg
            try:
                self.chip.configure(text="GUI ERROR - see console", bg=C["danger"])
            except Exception:
                pass
        finally:
            if not self._closing:
                self.root.after(self.TICK_MS, self.tick)

    def _publish_ros(self, c):
        if rclpy is not None and not rclpy.ok():
            self.ros_node = None
            return
        try:
            ok = self.ros_node.publish_joint_states(c.current_joint_angles, c.current_joint_velocities)
        except Exception as e:
            ok = False
            print(f"[GUI] ROS publish failed: {e}")
        if ok is False:
            print("[GUI] ROS context is not valid any more - ROS publishing disabled.")
            self.ros_node = None

    def _refresh_readouts(self, ang_limit):
        c = self.ctrl
        p = c.current_end_effector_pose
        for lbl, v in zip(self.pose_lbls, (p[0] * 1000, p[1] * 1000, p[2] * 1000, p[3])):
            lbl.configure(text=f"{v:7.1f}")
        v = np.array(c.current_tip_velocity) * 1000
        self.lbl_tip_v.configure(text=f"vx{v[0]:+6.1f}  vy{v[1]:+6.1f}  vz{v[2]:+6.1f}   |v| {np.linalg.norm(v):5.1f} mm/s")

        self.syncing = True
        for i in range(4):
            q, qd = c.current_joint_angles[i], c.current_joint_velocities[i]
            at_limit = abs(q) >= LIMIT - 0.5
            self.lbl_act[i].configure(text=f"{q:+6.1f}°", fg=C["danger"] if at_limit else C["text"])
            self.lbl_vel[i].configure(text=f"{qd:+6.1f}°/s")
            if i not in self.dragging:
                self.sliders[i].set(c.joint_targets[i])
        self.syncing = False

        colors = {"ok": C["ok"], "warn": C["warn"], "error": C["danger"]}
        self.chip.configure(text=c.status.upper(), bg=colors[c.status_level])
        self.lbl_mode.configure(text=c.mode)