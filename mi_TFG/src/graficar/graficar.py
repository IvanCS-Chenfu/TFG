#!/usr/bin/env python3
import threading
from collections import deque
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt8MultiArray

import matplotlib.pyplot as plt


def labels_from_uint8_2d(msg: UInt8MultiArray) -> List[str]:
    """
    Espera layout 2D:
      dim[0] = words (N)
      dim[1] = chars (M)
    data = N*M bytes, cada fila es una palabra (UTF-8/ASCII), relleno con 0.
    """
    if len(msg.layout.dim) < 2:
        return []

    n_words = int(msg.layout.dim[0].size)
    n_chars = int(msg.layout.dim[1].size)
    if n_words <= 0 or n_chars <= 0:
        return []

    expected = n_words * n_chars
    if len(msg.data) < expected:
        return []

    labels: List[str] = []
    data = msg.data
    for i in range(n_words):
        row = list(data[i * n_chars:(i + 1) * n_chars])
        if 0 in row:
            row = row[:row.index(0)]
        labels.append(bytes(row).decode("utf-8", errors="replace"))
    return labels


class MultiArrayPlotter(Node):
    def __init__(self):
        super().__init__("multiarray_plotter")

        # Parámetros
        self.declare_parameter("numeric_topic", "/numeric_array")
        self.declare_parameter("labels_topic", "/labels_array")
        self.declare_parameter("window_seconds", 10.0)     # 0 = todo el histórico
        self.declare_parameter("poll_publishers_hz", 5.0)

        self.numeric_topic = self.get_parameter("numeric_topic").value
        self.labels_topic = self.get_parameter("labels_topic").value
        self.window_seconds = float(self.get_parameter("window_seconds").value)
        self.poll_hz = float(self.get_parameter("poll_publishers_hz").value)

        # Subscriptores
        self.sub_num = self.create_subscription(Float64MultiArray, self.numeric_topic, self.on_numeric, 10)
        self.sub_lbl = self.create_subscription(UInt8MultiArray, self.labels_topic, self.on_labels, 10)

        # Timer: detectar si hay publishers
        self.create_timer(1.0 / max(self.poll_hz, 0.5), self.poll_publishers)

        # Estado
        self.running = False
        self.t0: Optional[Time] = None

        self.latest_labels: List[str] = []

        # Buffers
        self.t_buf = deque(maxlen=5000)
        self.series_buf: List[deque] = []
        self.expected_len: Optional[int] = None

        # Flags para el hilo principal (matplotlib)
        self._lock = threading.Lock()
        self._need_redraw = False
        self._need_reset_plot = True
        self._pending_init_len: Optional[int] = None

        # Matplotlib (solo hilo principal)
        self.fig, self.ax = plt.subplots()
        self.lines = []
        self._setup_axes()

        self.get_logger().info(
            f"Escuchando numeric={self.numeric_topic} (Float64MultiArray) "
            f"y labels={self.labels_topic} (UInt8MultiArray 2D). "
            f"Usa sim time: --ros-args -p use_sim_time:=true"
        )

    # ---------- Matplotlib helpers (solo main thread) ----------
    def _setup_axes(self):
        self.ax.set_title("MultiArray vs Sim Time")
        self.ax.set_xlabel("t (s) desde t0")
        self.ax.set_ylabel("valor")
        self.ax.grid(True)

    def _reset_plot_mainthread(self):
        self.ax.clear()
        self._setup_axes()
        self.lines = []
        if getattr(self.ax, "legend_", None) is not None:
            self.ax.legend_.remove()

    def _init_lines_mainthread(self, n: int):
        self.lines = []
        for i in range(n):
            (ln,) = self.ax.plot([], [], label=f"idx {i}")
            self.lines.append(ln)

    def _apply_legend_mainthread(self, n: int, labels: List[str]):
        if getattr(self.ax, "legend_", None) is not None:
            self.ax.legend_.remove()

        if len(labels) == n and n > 0:
            for i, ln in enumerate(self.lines):
                ln.set_label(labels[i])
            self.ax.legend(loc="upper right")

    # ---------- ROS callbacks (NO matplotlib aquí) ----------
    def on_labels(self, msg: UInt8MultiArray):
        labels = labels_from_uint8_2d(msg)
        with self._lock:
            self.latest_labels = labels
            self._need_redraw = True

    def on_numeric(self, msg: Float64MultiArray):
        with self._lock:
            if not self.running or self.t0 is None:
                return

            data = list(msg.data)
            n = len(data)
            if n == 0:
                return

            now = self.get_clock().now()
            t = (now - self.t0).nanoseconds * 1e-9

            # Primer mensaje o cambio de tamaño => reiniciar buffers + pedir init de líneas en main thread
            if self.expected_len is None:
                self.expected_len = n
                self.t_buf.clear()
                self.series_buf = [deque(maxlen=self.t_buf.maxlen) for _ in range(n)]
                self._pending_init_len = n
            elif n != self.expected_len:
                self.get_logger().warn(f"Tamaño del array cambió {self.expected_len} -> {n}. Reiniciando run.")
                self.expected_len = n
                self.t0 = now
                self.t_buf.clear()
                self.series_buf = [deque(maxlen=self.t_buf.maxlen) for _ in range(n)]
                self._pending_init_len = n

            # Guardar muestra (consistente)
            self.t_buf.append(t)
            for i, v in enumerate(data):
                self.series_buf[i].append(v)

            self._need_redraw = True

    def poll_publishers(self):
        pubs = self.get_publishers_info_by_topic(self.numeric_topic)
        has_pubs = len(pubs) > 0

        if has_pubs and not self.running:
            self.start_run()
        elif (not has_pubs) and self.running:
            self.stop_run()

    def start_run(self):
        with self._lock:
            self.running = True
            self.t0 = self.get_clock().now()

            self.t_buf.clear()
            self.series_buf = []
            self.expected_len = None

            self._pending_init_len = None
            self._need_reset_plot = True
            self._need_redraw = True

        self.get_logger().info(f"📈 Detectado publisher en {self.numeric_topic}. t0(sim) = {self.t0.nanoseconds} ns")

    def stop_run(self):
        with self._lock:
            self.running = False
        self.get_logger().info(f"⏸️ Sin publishers en {self.numeric_topic}. Pausando gráfica.")

    # ---------- Matplotlib update (SOLO main thread) ----------
    def mpl_update(self):
        # snapshot mínimo bajo lock
        with self._lock:
            need_reset = self._need_reset_plot
            self._need_reset_plot = False

            need_redraw = self._need_redraw
            self._need_redraw = False

            pending_init_len = self._pending_init_len
            self._pending_init_len = None

            n = self.expected_len
            labels = list(self.latest_labels)
            t_list = list(self.t_buf)
            series_copy = [list(d) for d in self.series_buf]

        if not need_redraw and not need_reset and pending_init_len is None:
            return

        if need_reset:
            self._reset_plot_mainthread()

        if pending_init_len is not None:
            self._reset_plot_mainthread()
            self._init_lines_mainthread(pending_init_len)
            self._apply_legend_mainthread(pending_init_len, labels)

        if n is None or len(t_list) == 0 or len(self.lines) == 0:
            self.fig.canvas.draw_idle()
            return

        # --- calcular ventana visible en X (OPTION 2) ---
        if self.window_seconds > 0.0:
            tmax = t_list[-1]
            tmin = max(0.0, tmax - self.window_seconds)
            # índice inicial visible
            start = 0
            # búsqueda lineal (suficiente para 5000 puntos). Si quieres, lo optimizo con bisect.
            for i, tt in enumerate(t_list):
                if tt >= tmin:
                    start = i
                    break
            t_vis = t_list[start:]
            self.ax.set_xlim(tmin, max(tmax, tmin + 1e-6))
        else:
            start = 0
            t_vis = t_list
            self.ax.set_xlim(0.0, max(t_list[-1], 1e-6))

        # actualizar líneas con el mismo tramo visible
        y_min = float("inf")
        y_max = float("-inf")

        for i, ln in enumerate(self.lines):
            if i >= len(series_copy):
                continue
            y_full = series_copy[i]
            if len(y_full) == 0:
                continue

            # asegurar longitudes consistentes (por seguridad)
            m = min(len(t_list), len(y_full))
            t_cut = t_list[-m:]
            y_cut = y_full[-m:]

            # recortar al tramo visible (start relativo a t_cut si hubo recorte por m)
            # pero como t_cut/y_cut están alineados, calculamos start_cut en ese espacio:
            if self.window_seconds > 0.0 and len(t_cut) > 0:
                # recomputar start en el vector recortado
                tmax2 = t_cut[-1]
                tmin2 = max(0.0, tmax2 - self.window_seconds)
                start_cut = 0
                for k, tt in enumerate(t_cut):
                    if tt >= tmin2:
                        start_cut = k
                        break
                t_plot = t_cut[start_cut:]
                y_plot = y_cut[start_cut:]
            else:
                t_plot = t_cut
                y_plot = y_cut

            ln.set_data(t_plot, y_plot)

            # --- autoscale Y SOLO con lo visible ---
            if len(y_plot) > 0:
                y_min = min(y_min, min(y_plot))
                y_max = max(y_max, max(y_plot))

        # Y-lim dinámico real (solo ventana visible)
        if y_min < y_max:
            margin = 0.05 * (y_max - y_min)
            self.ax.set_ylim(y_min - margin, y_max + margin)
        elif y_min == y_max and y_min not in (float("inf"), float("-inf")):
            eps = 1e-3 if abs(y_min) < 1 else abs(y_min) * 0.01
            self.ax.set_ylim(y_min - eps, y_max + eps)

        # leyenda (por si cambian labels)
        self._apply_legend_mainthread(n, labels)

        self.fig.canvas.draw_idle()


def main():
    rclpy.init()
    node = MultiArrayPlotter()

    # ROS spin en hilo aparte
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Matplotlib en hilo principal
    plt.ion()
    timer = node.fig.canvas.new_timer(interval=50)  # ms
    timer.add_callback(node.mpl_update)
    timer.start()

    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
