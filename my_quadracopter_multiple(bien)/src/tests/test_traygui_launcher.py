#!/usr/bin/env python3
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Pose
from my_quadracopter.action import Poly3Traj  # asegúrate de que el paquete exporta la acción


class Poly3TrajGUI(Node):
    """
    Nodo ROS 2 con GUI (tkinter) para lanzar la acción Poly3Traj a /drone_i/poly3_traj.
    - Lee el parámetro 'number_of_drones' (por defecto 1).
    - 9 sliders: x, y, z, yaw, tx, ty, tz, tyaw, drone_index.
    - Botón 'Enviar' que manda el goal con los valores actuales.
    """

    def __init__(self):
        super().__init__('poly3_traj_gui')

        # --- Parámetros ---
        self.declare_parameter('number_of_drones', 1)
        self.number_of_drones = max(
            1, int(self.get_parameter('number_of_drones').get_parameter_value().integer_value)
        )

        # Valores por defecto
        self.defaults = {
            'x': 0.0, 'y': 0.0, 'z': 1.0, 'yaw_deg': 0.0,
            'tx': 20.0, 'ty': 20.0, 'tz': 20.0, 'tyaw': 20.0,
            'drone_idx': 0
        }

        # --- Clientes de acción (uno por namespace, creados bajo demanda) ---
        self._clients = {}  # action_name -> ActionClient

        # --- Construir GUI ---
        self._build_gui()

        self.get_logger().info(
            f"GUI de Poly3Traj lista. Drones disponibles: 0..{self.number_of_drones - 1}."
        )

    # ---------------- GUI ----------------
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("Poly3Traj Launcher")

        frm = ttk.Frame(self.root, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Variables tkinter
        self.var_x     = tk.DoubleVar(value=self.defaults['x'])
        self.var_y     = tk.DoubleVar(value=self.defaults['y'])
        self.var_z     = tk.DoubleVar(value=self.defaults['z'])
        self.var_yaw   = tk.DoubleVar(value=self.defaults['yaw_deg'])

        self.var_tx    = tk.DoubleVar(value=self.defaults['tx'])
        self.var_ty    = tk.DoubleVar(value=self.defaults['ty'])
        self.var_tz    = tk.DoubleVar(value=self.defaults['tz'])
        self.var_tyaw  = tk.DoubleVar(value=self.defaults['tyaw'])

        self.var_drone = tk.IntVar(value=min(self.defaults['drone_idx'], self.number_of_drones - 1))

        # Helper para sliders (continuos)
        def add_slider(row, text, var, frm_range, step, fmt="{:.2f}"):
            lbl = ttk.Label(frm, text=text, width=20)
            lbl.grid(row=row, column=0, sticky="w", pady=4)
            val_lbl = ttk.Label(frm, text=fmt.format(var.get()), width=8)
            val_lbl.grid(row=row, column=2, sticky="e")
            sld = ttk.Scale(frm, from_=frm_range[0], to=frm_range[1], orient="horizontal",
                            variable=var, command=lambda _evt: val_lbl.config(text=fmt.format(var.get())))
            sld.grid(row=row, column=1, sticky="we", padx=8)
            frm.columnconfigure(1, weight=1)

            # ajuste fino con la rueda del ratón
            def on_mouse_wheel(event, v=var, st=step):
                delta = st if event.delta > 0 else -st
                v.set(max(frm_range[0], min(frm_range[1], v.get() + delta)))
                val_lbl.config(text=fmt.format(v.get()))
            sld.bind("<MouseWheel>", on_mouse_wheel)
            return sld

        r = 0
        add_slider(r, "x  (m)", self.var_x,   (-10.0, 10.0), 0.1); r += 1
        add_slider(r, "y  (m)", self.var_y,   (-10.0, 10.0), 0.1); r += 1
        add_slider(r, "z  (m)", self.var_z,   (0.0,   10.0), 0.1); r += 1
        add_slider(r, "yaw (deg)", self.var_yaw, (-180.0, 180.0), 1.0, fmt="{:.0f}"); r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=3, sticky="we", pady=(6,6)); r += 1

        add_slider(r, "tx (s)", self.var_tx, (1.0, 60.0), 1.0, fmt="{:.0f}"); r += 1
        add_slider(r, "ty (s)", self.var_ty, (1.0, 60.0), 1.0, fmt="{:.0f}"); r += 1
        add_slider(r, "tz (s)", self.var_tz, (1.0, 60.0), 1.0, fmt="{:.0f}"); r += 1
        add_slider(r, "tyaw (s)", self.var_tyaw, (1.0, 60.0), 1.0, fmt="{:.0f}"); r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=3, sticky="we", pady=(6,12)); r += 1

        # Slider de dron (entero); ttk.Scale no tiene 'resolution', así que redondeamos
        lbl_dr = ttk.Label(frm, text=f"drone index (0..{self.number_of_drones-1})", width=28)
        lbl_dr.grid(row=r, column=0, sticky="w", pady=4)
        val_dr = ttk.Label(frm, text=str(self.var_drone.get()), width=8)
        val_dr.grid(row=r, column=2, sticky="e")
        sld_dr = ttk.Scale(
            frm, from_=0, to=max(0, self.number_of_drones - 1),
            orient="horizontal", variable=self.var_drone,
            command=lambda _evt: val_dr.config(text=str(int(round(self.var_drone.get()))))
        )
        sld_dr.grid(row=r, column=1, sticky="we", padx=8)
        frm.columnconfigure(1, weight=1)
        self._drone_scale = sld_dr  # por si quieres actualizar límites más tarde
        r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=3, sticky="we", pady=(6,12)); r += 1

        # Botón enviar
        btn = ttk.Button(frm, text="Enviar", command=self._on_send)
        btn.grid(row=r, column=0, columnspan=3, sticky="we")

        # Cerrar limpio
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------- Lógica de acción -------------
    def _get_client(self, drone_ns: str) -> ActionClient:
        """ Devuelve/crea ActionClient para /<drone_ns>/poly3_traj """
        action_name = f"/{drone_ns}/poly3_traj"
        if action_name not in self._clients:
            self._clients[action_name] = ActionClient(self, Poly3Traj, action_name)
        return self._clients[action_name]

    def _on_send(self):
        # Recoger valores
        x   = float(self.var_x.get())
        y   = float(self.var_y.get())
        z   = float(self.var_z.get())
        yaw_deg = float(self.var_yaw.get())
        tx  = float(self.var_tx.get())
        ty  = float(self.var_ty.get())
        tz  = float(self.var_tz.get())
        tyaw= float(self.var_tyaw.get())

        idx = int(round(self.var_drone.get()))
        if not (0 <= idx < self.number_of_drones):
            messagebox.showerror("Índice de dron", f"Índice fuera de rango: {idx}")
            return
        drone_ns = f"drone_{idx}"

        # Convertir yaw grados -> quaternion sobre Z
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        # Preparar goal
        goal = Poly3Traj.Goal()
        goal.target_pose = Pose()
        goal.target_pose.position.x = x
        goal.target_pose.position.y = y
        goal.target_pose.position.z = z
        goal.target_pose.orientation.x = 0.0
        goal.target_pose.orientation.y = 0.0
        goal.target_pose.orientation.z = qz
        goal.target_pose.orientation.w = qw
        goal.tx = float(tx)
        goal.ty = float(ty)
        goal.tz = float(tz)
        goal.tyaw = float(tyaw)

        client = self._get_client(drone_ns)

        # Enviar en hilo aparte para no bloquear la GUI
        def send():
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Servidor de acción no disponible: /{drone_ns}/poly3_traj")
                self._toast(f"No disponible: /{drone_ns}/poly3_traj")
                return

            self.get_logger().info(
                f"Enviando goal a /{drone_ns}/poly3_traj: "
                f"pos=({x:.2f},{y:.2f},{z:.2f}), yaw={yaw_deg:.1f}deg, "
                f"tx={tx:.0f}, ty={ty:.0f}, tz={tz:.0f}, tyaw={tyaw:.0f}"
            )

            # 1) enviar el goal (asíncrono)
            send_future = client.send_goal_async(
                goal,
                feedback_callback=lambda fb: None  # aquí podrías mostrar progreso
            )

            # 2) cuando el servidor responde (aceptado/rechazado)
            def _goal_response_cb(fut):
                goal_handle = fut.result()
                if not goal_handle.accepted:
                    self.get_logger().error("Goal rechazado")
                    self._toast("Goal rechazado")
                    return

                self.get_logger().info("Goal aceptado")

                # 3) esperar el resultado
                result_future = goal_handle.get_result_async()

                def _result_cb(rf):
                    result = rf.result()
                    status = result.status
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        t_total = getattr(result.result, 't_total', float('nan'))
                        msg = f"SUCCEEDED ({drone_ns}), t_total={t_total:.3f}"
                        self.get_logger().info(msg)
                        self._toast(msg)
                    else:
                        msg = f"Acción terminó con código {int(status)} en {drone_ns}"
                        self.get_logger().warn(msg)
                        self._toast(msg)

                result_future.add_done_callback(_result_cb)

            send_future.add_done_callback(_goal_response_cb)

        threading.Thread(target=send, daemon=True).start()

    def _toast(self, text: str):
        try:
            self.root.after(0, lambda: messagebox.showinfo("Poly3Traj", text))
        except tk.TclError:
            pass  # ventana ya cerrada

    def _on_close(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        rclpy.shutdown()


def main():
    # tkinter en hilo principal; rclpy en otro hilo
    rclpy.init()
    node = Poly3TrajGUI()

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
