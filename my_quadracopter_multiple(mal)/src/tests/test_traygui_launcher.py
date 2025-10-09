#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel,
    QSlider, QPushButton, QHBoxLayout, QFrame
)
from PyQt5.QtCore import Qt

from geometry_msgs.msg import PoseStamped
from my_quadracopter.action import Poly3Traj


def yaw_to_quat_zw(yaw_rad: float):
    """Devuelve (z, w) para un yaw en radianes (rotación alrededor de Z)."""
    z = math.sin(yaw_rad * 0.5)
    w = math.cos(yaw_rad * 0.5)
    return z, w


class Poly3GuiNode(Node):
    def __init__(self):
        super().__init__('poly3_traj_gui_launcher')

        # Parámetros
        self.declare_parameter('action_name', '/poly3_traj')
        self.declare_parameter('ns_prefix', '/drone_')
        self.declare_parameter('num_drones', 1)

        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.ns_prefix   = self.get_parameter('ns_prefix').get_parameter_value().string_value
        self.num_drones  = self.get_parameter('num_drones').get_parameter_value().integer_value
        if self.num_drones < 1:
            self.num_drones = 1

        self.client = ActionClient(self, Poly3Traj, self.action_name)

        self.get_logger().info(f"Esperando servidor de acción: {self.action_name} ...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Servidor no disponible aún; podrás pulsar el botón cuando esté listo.")

        # --- GUI ---
        self.app = QApplication.instance() or QApplication(sys.argv)
        self.win = QWidget()
        self.win.setWindowTitle('Poly3Traj – GUI')

        root = QVBoxLayout(self.win)

        grid = QGridLayout()
        row = 0

        # Helper para crear sliders + etiqueta
        def add_slider(label_text, minv, maxv, init, step=1):
            nonlocal row
            lbl = QLabel(f"{label_text}: {init}")
            sld = QSlider(Qt.Horizontal)
            sld.setMinimum(minv)
            sld.setMaximum(maxv)
            sld.setSingleStep(step)
            sld.setValue(init)
            sld.valueChanged.connect(lambda v, l=lbl, t=label_text: l.setText(f"{t}: {v}"))
            grid.addWidget(lbl, row, 0)
            grid.addWidget(sld, row, 1)
            row += 1
            return sld, lbl

        # 1) Posiciones y yaw
        self.sld_x, _   = add_slider("x [-20..20] m",   -20, 20, 0, 1)
        self.sld_y, _   = add_slider("y [-20..20] m",   -20, 20, 0, 1)
        self.sld_z, _   = add_slider("z [0..20] m",       0, 20,  1, 1)
        self.sld_yaw, _ = add_slider("yaw [-180..180]°", -180, 180, 0, 1)

        # Separador
        sep = QFrame(); sep.setFrameShape(QFrame.HLine); sep.setFrameShadow(QFrame.Sunken)
        grid.addWidget(sep, row, 0, 1, 2); row += 1

        # 2) Tiempos
        self.sld_tx, _ = add_slider("tx [1..60] s", 1, 60, 5, 1)
        self.sld_ty, _ = add_slider("ty [1..60] s", 1, 60, 5, 1)
        self.sld_tz, _ = add_slider("tz [1..60] s", 1, 60, 15, 1)
        self.sld_tyaw, _ = add_slider("tyaw [1..60] s", 1, 60, 5, 1)

        # Separador
        sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine); sep2.setFrameShadow(QFrame.Sunken)
        grid.addWidget(sep2, row, 0, 1, 2); row += 1

        # 3) ID de dron (0 .. num_drones-1)
        self.sld_drone, self.lbl_drone = add_slider(f"drone_id [0..{self.num_drones-1}]",
                                                    0, self.num_drones-1, 0, 1)

        root.addLayout(grid)

        # Botón enviar
        btn_layout = QHBoxLayout()
        self.btn_send = QPushButton("Enviar goal")
        self.btn_send.clicked.connect(self.on_send)
        btn_layout.addStretch(1)
        btn_layout.addWidget(self.btn_send)
        root.addLayout(btn_layout)

        self.win.resize(520, 360)
        self.win.show()

    def spin_gui(self):
        # Integramos el loop de ROS y Qt
        timer = self.create_timer(0.02, lambda: None)  # 50 Hz “no-op” para rclpy spin
        self.app.exec_()
        timer.cancel()

    def on_send(self):
        if not self.client.server_is_ready():
            self.get_logger().warn("Servidor de acción no está listo todavía.")
            return

        x   = float(self.sld_x.value())
        y   = float(self.sld_y.value())
        z   = float(self.sld_z.value())
        yaw_deg = float(self.sld_yaw.value())
        tx  = float(self.sld_tx.value())
        ty  = float(self.sld_ty.value())
        tz  = float(self.sld_tz.value())
        tyaw= float(self.sld_tyaw.value())
        drone_id = int(self.sld_drone.value())

        ns = f"{self.ns_prefix}{drone_id}"
        yaw_rad = math.radians(yaw_deg)
        qz, qw = yaw_to_quat_zw(yaw_rad)

        goal = Poly3Traj.Goal()
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.header.frame_id = ns
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        goal.tx = float(tx); goal.ty = float(ty); goal.tz = float(tz); goal.tyaw = float(tyaw)

        self.get_logger().info(
            f"Enviando goal a {self.action_name} -> ns={ns} | "
            f"pos=({x:.1f},{y:.1f},{z:.1f}) yaw={yaw_deg:.1f}° "
            f"t=({tx:.1f},{ty:.1f},{tz:.1f},{tyaw:.1f})"
        )

        def goal_response_cb(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().error("Goal rechazado")
                return
            self.get_logger().info("Goal aceptado")
            gh.get_result_async().add_done_callback(result_cb)

        def feedback_cb(fb):
            # opcional: imprimir feedback
            pass

        def result_cb(fut):
            res = fut.result().result
            self.get_logger().info(f"Resultado: success={res.success} t_total={res.t_total:.3f}s")

        future = self.client.send_goal_async(goal, feedback_callback=feedback_cb)
        future.add_done_callback(goal_response_cb)



def main():
    rclpy.init()
    node = Poly3GuiNode()
    try:
        node.spin_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
