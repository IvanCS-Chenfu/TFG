#!/usr/bin/env python3
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from mi_tfg.action import TrayAction


ACTION_NAME = "AccionTrayectoria"   # <-- pon aquí el nombre EXACTO del servidor de acción


class TrayActionGUI(Node):
    def __init__(self):
        super().__init__("tray_action_gui")

        # ActionClient único (sin diccionarios ni namespaces)
        self._client = ActionClient(self, TrayAction, ACTION_NAME)

        # Defaults
        self.defaults = {
            "x": 0.0, "y": 0.0, "z": 1.0, "yaw_deg": 0.0,
            "tx": 20.0, "ty": 20.0, "tz": 20.0, "tyaw": 20.0,
        }

        self._build_gui()
        self.get_logger().info(f"GUI lista. Action server: {ACTION_NAME}")

    # ---------------- GUI ----------------
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("TrayAction Launcher (minimal)")

        frm = ttk.Frame(self.root, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        self.var_x    = tk.DoubleVar(value=self.defaults["x"])
        self.var_y    = tk.DoubleVar(value=self.defaults["y"])
        self.var_z    = tk.DoubleVar(value=self.defaults["z"])
        self.var_yaw  = tk.DoubleVar(value=self.defaults["yaw_deg"])
        self.var_tx   = tk.DoubleVar(value=self.defaults["tx"])
        self.var_ty   = tk.DoubleVar(value=self.defaults["ty"])
        self.var_tz   = tk.DoubleVar(value=self.defaults["tz"])
        self.var_tyaw = tk.DoubleVar(value=self.defaults["tyaw"])

        def add_slider(row, text, var, frm_range, fmt="{:.2f}"):
            ttk.Label(frm, text=text, width=14).grid(row=row, column=0, sticky="w", pady=4)
            val_lbl = ttk.Label(frm, text=fmt.format(var.get()), width=8)
            val_lbl.grid(row=row, column=2, sticky="e")
            sld = ttk.Scale(
                frm, from_=frm_range[0], to=frm_range[1], orient="horizontal",
                variable=var, command=lambda _evt: val_lbl.config(text=fmt.format(var.get()))
            )
            sld.grid(row=row, column=1, sticky="we", padx=8)

        r = 0
        add_slider(r, "x (m)", self.var_x, (-10.0, 10.0)); r += 1
        add_slider(r, "y (m)", self.var_y, (-10.0, 10.0)); r += 1
        add_slider(r, "z (m)", self.var_z, (0.0, 10.0)); r += 1
        add_slider(r, "yaw (deg)", self.var_yaw, (-180.0, 180.0), fmt="{:.0f}"); r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=3, sticky="we", pady=(6, 6)); r += 1

        add_slider(r, "tx (s)", self.var_tx, (1.0, 60.0), fmt="{:.0f}"); r += 1
        add_slider(r, "ty (s)", self.var_ty, (1.0, 60.0), fmt="{:.0f}"); r += 1
        add_slider(r, "tz (s)", self.var_tz, (1.0, 60.0), fmt="{:.0f}"); r += 1
        add_slider(r, "tyaw (s)", self.var_tyaw, (1.0, 60.0), fmt="{:.0f}"); r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=3, sticky="we", pady=(10, 10)); r += 1

        ttk.Button(frm, text="Enviar goal", command=self._on_send).grid(row=r, column=0, columnspan=3, sticky="we")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------- Acción -------------
    def _on_send(self):
        # Leer sliders
        x = float(self.var_x.get())
        y = float(self.var_y.get())
        z = float(self.var_z.get())
        yaw_deg = float(self.var_yaw.get())
        tx = float(self.var_tx.get())
        ty = float(self.var_ty.get())
        tz = float(self.var_tz.get())
        tyaw = float(self.var_tyaw.get())

        # yaw deg -> quaternion (Z)
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        # Construir goal
        goal = TrayAction.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        goal.tx = tx
        goal.ty = ty
        goal.tz = tz
        goal.tyaw = tyaw

        # Enviar (en hilo para no bloquear tkinter)
        def send_goal_thread():
            if not self._client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(f"No hay servidor de acción en {ACTION_NAME}")
                return

            self.get_logger().info(
                f"Enviando goal a {ACTION_NAME}: "
                f"pos=({x:.2f},{y:.2f},{z:.2f}), yaw={yaw_deg:.1f}deg, "
                f"tx={tx:.1f}, ty={ty:.1f}, tz={tz:.1f}, tyaw={tyaw:.1f}"
            )

            # Solo enviar (sin callbacks extra para minimizar problemas)
            self._client.send_goal_async(goal)

        threading.Thread(target=send_goal_thread, daemon=True).start()

    def _on_close(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrayActionGUI()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
