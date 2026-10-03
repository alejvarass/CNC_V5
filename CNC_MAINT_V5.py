import sys
import json
import math
import serial
import serial.tools.list_ports

from PySide6.QtCore import Qt, QThread, Signal, QTimer, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QGroupBox, QTableWidget,
    QTableWidgetItem, QMessageBox, QHeaderView, QSpinBox, QSizePolicy
)

# Diccionario de ayuda contextual para los encabezados de la tabla NVS (V5)
NVS_HELP_DESCRIPTIONS = {
    0: ("Eje", "Identificador físico del eje cartesiano o auxiliar (X, Y, Z, W)."),
    1: ("SPM (Steps/mm)", "Resolución cinemática: cantidad de pulsos para avanzar 1.0 mm real."),
    2: ("Max Travel (mm)", "Carrera útil máxima permitida por software (Soft Limit)."),
    3: ("Backoff (stp)", "Pasos que retrocede el carro tras tocar el sensor en búsqueda rápida de Home."),
    4: ("SoftOff (stp)", "Pasos de separación para fijar el origen instrumental (0.000 mm)."),
    5: ("Curva S (1/0)", "Modo de aceleración: '1' con rampa cosenoidal suave; '0' velocidad constante."),
    6: ("Seek (us)", "Periodo entre pulsos para búsqueda rápida de Home (mínimo 80 us)."),
    7: ("Feed (us)", "Periodo entre pulsos para aproximación fina de Home (mínimo 80 us)."),
    8: ("BO us", "Periodo entre pulsos en retroceso de desenganche de Home."),
    9: ("Man us", "Periodo entre pulsos para desplazamiento manual continuo."),
    10: ("Jog us", "Periodo entre pulsos para movimientos incrementales fijos."),
    11: ("Dir Forward", "Nivel lógico en pin DIR para sentido positivo ('1'=HIGH, '0'=LOW).")
}


class DPadPainter(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)
    stopPressed = Signal()

    def __init__(self, labels=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(110, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pressed = False
        self._active = None
        self.setMouseTracking(False)  # Evita activación por mero movimiento de cursor
        self.labels = labels or {"up": "Y+", "down": "Y-", "left": "X-", "right": "X+"}

    def _geom(self):
        w, h = self.width(), self.height()
        dim = min(w, h)
        c = QPointF(w / 2.0, h / 2.0)
        outer = dim * 0.44
        inner = dim * 0.22
        return c, outer, inner, inner

    def _hit(self, pos):
        c, outer, inner, stop_r = self._geom()
        x = pos.x() - c.x()
        y = pos.y() - c.y()
        r = math.hypot(x, y)
        if r <= stop_r: return "stop"
        if r > outer: return None
        ang = math.degrees(math.atan2(-y, x))
        if ang < 0: ang += 360
        if 45 <= ang < 135: return "up"
        if 135 <= ang < 225: return "left"
        if 225 <= ang < 315: return "down"
        return "right"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton: return
        zone = self._hit(e.position())
        if zone is None: return
        self._pressed = True
        self._active = zone
        if zone == "stop": self.stopPressed.emit()
        else: self.directionPressed.emit(zone)
        self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed or not (e.buttons() & Qt.LeftButton): 
            return
        zone = self._hit(e.position())
        if zone != self._active:
            old_zone = self._active
            self._active = zone
            if old_zone and old_zone != "stop": self.directionReleased.emit(old_zone)
            if zone == "stop": self.stopPressed.emit()
            elif zone is not None: self.directionPressed.emit(zone)
            self.update()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton and self._pressed:
            old = self._active
            self._pressed = False
            self._active = None
            if old and old != "stop": self.directionReleased.emit(old)
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c, outer, inner, stop_r = self._geom()
        p.fillRect(self.rect(), QColor("#161b22"))
        p.setPen(QPen(QColor("#2f4953"), max(1, int(outer * 0.015))))
        p.setBrush(QColor("#0f1a20"))
        p.drawEllipse(c, outer, outer)
        gap = max(2.5, outer * 0.04)

        def draw_sector(start_deg, span_deg, key):
            outer_rect = QRectF(c.x() - outer, c.y() - outer, outer * 2, outer * 2)
            inner_rect = QRectF(c.x() - inner, c.y() - inner, inner * 2, inner * 2)
            path = QPainterPath()
            path.arcMoveTo(outer_rect, start_deg + gap / 2.0)
            path.arcTo(outer_rect, start_deg + gap / 2.0, span_deg - gap)
            path.arcTo(inner_rect, start_deg + span_deg - gap / 2.0, -(span_deg - gap))
            path.closeSubpath()
            p.setPen(QPen(QColor("#2f4953"), max(1, int(outer * 0.015))))
            p.setBrush(QColor("#1f4958") if self._active == key else (QColor("#12232c") if self.isEnabled() else QColor("#1c2128")))
            p.drawPath(path)

        draw_sector(45, 90, "up")
        draw_sector(135, 90, "left")
        draw_sector(225, 90, "down")
        draw_sector(315, 90, "right")

        p.setPen(QPen(QColor("#b02a2a"), max(2, int(stop_r * 0.06))))
        p.setBrush(QColor("#a42424") if self._active == "stop" else QColor("#7d1b1b"))
        p.drawEllipse(c, stop_r, stop_r)
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        
        f_size = max(8, int(inner * 0.40))
        p.setFont(QFont("Segoe UI", f_size, QFont.Bold))
        lbl_w = outer * 0.6
        lbl_h = max(16, int(inner * 0.5))
        p.drawText(QRectF(c.x() - lbl_w / 2, c.y() - outer + (outer - inner) * 0.15, lbl_w, lbl_h), Qt.AlignCenter, self.labels.get("up", "↑"))
        p.drawText(QRectF(c.x() - outer + (outer - inner) * 0.1, c.y() - lbl_h / 2, lbl_w, lbl_h), Qt.AlignCenter, self.labels.get("left", "←"))
        p.drawText(QRectF(c.x() + inner + (outer - inner) * 0.25, c.y() - lbl_h / 2, lbl_w, lbl_h), Qt.AlignCenter, self.labels.get("right", "→"))
        p.drawText(QRectF(c.x() - lbl_w / 2, c.y() + inner + (outer - inner) * 0.35, lbl_w, lbl_h), Qt.AlignCenter, self.labels.get("down", "↓"))

        p.setFont(QFont("Segoe UI", max(7, int(stop_r * 0.34)), QFont.Bold))
        p.drawText(QRectF(c.x() - stop_r, c.y() - stop_r, stop_r * 2, stop_r * 2), Qt.AlignCenter, "STOP")
        p.end()


class VerticalZWControl(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(90, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pressed = False
        self._active = None
        self.setMouseTracking(False)

    def _get_outer_oval_rect(self):
        w, h = self.width(), self.height()
        oval_w = min(w * 0.76, h * 0.55)
        oval_h = min(h * 0.88, oval_w * 1.55)
        return QRectF((w - oval_w) / 2.0, (h - oval_h) / 2.0, oval_w, oval_h)

    def _get_inner_oval_rect(self, outer_r, gap):
        h_sec = (outer_r.height() - gap * 3) / 4.0
        top_w = outer_r.top() + h_sec + gap
        return QRectF(outer_r.left() + outer_r.width() * 0.08, top_w, outer_r.width() * 0.84, (h_sec * 2) + gap)

    def _hit(self, pos):
        outer_r = self._get_outer_oval_rect()
        if not outer_r.contains(pos): return None
        gap = max(2.5, outer_r.width() * 0.035)
        inner_r = self._get_inner_oval_rect(outer_r, gap)
        if inner_r.contains(pos): return "w_up" if pos.y() < inner_r.center().y() else "w_down"
        return "z_up" if pos.y() < outer_r.center().y() else "z_down"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton: return
        zone = self._hit(e.position())
        if zone:
            self._pressed = True; self._active = zone
            self.directionPressed.emit(zone)
            self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed or not (e.buttons() & Qt.LeftButton): 
            return
        zone = self._hit(e.position())
        if zone != self._active:
            if self._active: self.directionReleased.emit(self._active)
            self._active = zone
            if zone: self.directionPressed.emit(zone)
            self.update()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton and self._pressed:
            if self._active: self.directionReleased.emit(self._active)
            self._pressed = False; self._active = None
            self.update()

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#161b22"))
        outer_r = self._get_outer_oval_rect()
        gap = max(2.5, outer_r.width() * 0.035)
        inner_r = self._get_inner_oval_rect(outer_r, gap)

        bg_out = QPainterPath(); bg_out.addRoundedRect(outer_r, outer_r.width() / 2, outer_r.width() / 2)
        bg_in = QPainterPath(); bg_in.addRoundedRect(inner_r, inner_r.width() / 2, inner_r.width() / 2)
        p.setPen(QPen(QColor("#2f4953"), max(1, int(outer_r.width() * 0.015))))
        p.setBrush(QColor("#0f1a20")); p.drawPath(bg_out)

        color_idle = QColor("#12232c") if self.isEnabled() else QColor("#1c2128")
        color_act = QColor("#1f4958")
        h_sec = (outer_r.height() - gap * 3) / 4.0

        sec_z_up = QRectF(outer_r.left(), outer_r.top(), outer_r.width(), h_sec)
        path_z_up = QPainterPath(); path_z_up.addRect(sec_z_up)
        p.setBrush(color_act if self._active == "z_up" else color_idle); p.drawPath(path_z_up.intersected(bg_out))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", max(8, int(h_sec * 0.42)), QFont.Bold))
        p.drawText(sec_z_up, Qt.AlignCenter, "Z+")

        sec_z_down = QRectF(outer_r.left(), outer_r.top() + (h_sec + gap) * 3, outer_r.width(), h_sec)
        path_z_down = QPainterPath(); path_z_down.addRect(sec_z_down)
        p.setBrush(color_act if self._active == "z_down" else color_idle); p.drawPath(path_z_down.intersected(bg_out))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.drawText(sec_z_down, Qt.AlignCenter, "Z-")

        p.setPen(QPen(QColor("#00f0ff") if self.isEnabled() else QColor("#2f4953"), 1, Qt.DashLine))
        p.setBrush(QColor("#0d181e")); p.drawPath(bg_in)

        half_inner = (inner_r.height() - gap) / 2.0
        sec_w_up = QRectF(inner_r.left(), inner_r.top(), inner_r.width(), half_inner)
        path_w_up = QPainterPath(); path_w_up.addRect(sec_w_up)
        p.setBrush(color_act if self._active == "w_up" else color_idle); p.drawPath(path_w_up.intersected(bg_in))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", max(7, int(half_inner * 0.46)), QFont.Bold))
        p.drawText(sec_w_up, Qt.AlignCenter, "W+")

        sec_w_down = QRectF(inner_r.left(), inner_r.top() + half_inner + gap, inner_r.width(), half_inner)
        path_w_down = QPainterPath(); path_w_down.addRect(sec_w_down)
        p.setBrush(color_act if self._active == "w_down" else color_idle); p.drawPath(path_w_down.intersected(bg_in))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.drawText(sec_w_down, Qt.AlignCenter, "W-")
        p.end()


class SerialWorker(QThread):
    lineReceived = Signal(str)
    connected = Signal()
    disconnected = Signal(str)

    def __init__(self):
        super().__init__()
        self.port_name = ""
        self.baudrate = 115200
        self.ser = None
        self.running = False

    def configure(self, port: str, baud: int = 115200):
        self.port_name = port
        self.baudrate = baud

    def run(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            self.running = True
            self.connected.emit()
            while self.running:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='replace').strip()
                        if line:
                            self.lineReceived.emit(line)
                    except Exception:
                        pass
                self.msleep(10)
        except Exception as e:
            self.disconnected.emit(str(e))
        finally:
            self._close_port()

    def send_line(self, line: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((line.strip() + "\n").encode('utf-8'))
            except Exception:
                pass

    def stop(self):
        self.running = False
        self.wait(200)
        self._close_port()

    def _close_port(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


class MaintenanceWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CNC ESP32 V5 - Mantenimiento NVS & Control Libre USB")
        self.setMinimumSize(1150, 750)
        self.resize(1220, 780)
        self.worker = None
        self.nvs_cache = {}

        self.jog_timer = QTimer(self)
        self.jog_timer.setInterval(50)
        self.jog_timer.timeout.connect(self._send_jog_heartbeat)
        self._current_jog_cmd = None

        self._build_ui()
        self._apply_style()
        self._refresh_ports()

    def _apply_style(self):
        accent = "#006c6c"
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ 
                background-color: #161b22; 
                color: #dbe2ea; 
                font-family: "Segoe UI"; 
                font-size: 11px; 
            }}
            QGroupBox {{ 
                border: 1px solid #2d333b; 
                border-radius: 6px; 
                margin-top: 6px; 
                font-weight: bold; 
                color: {accent}; 
                padding-top: 8px; 
            }}
            QGroupBox::title {{ 
                subcontrol-origin: margin; 
                left: 8px; 
                padding: 0 4px; 
            }}
            QLineEdit, QComboBox, QTableWidget {{ 
                background-color: #0d1117; 
                border: 1px solid #30363d; 
                border-radius: 4px; 
                padding: 3px; 
                color: #e6edf3; 
            }}
            QPushButton {{ 
                background-color: {accent}; 
                border: 1px solid {accent}; 
                border-radius: 4px; 
                padding: 4px 8px; 
                font-weight: bold; 
                color: white; 
            }}
            QPushButton:hover {{ 
                background-color: #008080; 
                border-color: #008080; 
            }}
            QPushButton:disabled {{ 
                background-color: #21262d !important; 
                border: 1px solid #30363d !important; 
                color: #484f58 !important; 
            }}
            QHeaderView::section {{ 
                background-color: #0d1117; 
                color: #58a6ff; 
                font-weight: bold; 
                border: 1px solid #30363d; 
                padding: 4px; 
            }}
            QToolTip {{ 
                background-color: #fff8c5; 
                color: #24292f; 
                border: 1px solid #d4a72c; 
                border-radius: 4px; 
                padding: 6px 8px; 
                font-size: 11px; 
                font-weight: normal; 
            }}
        """)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_h_layout = QHBoxLayout(root)
        main_h_layout.setContentsMargins(8, 8, 8, 8)
        main_h_layout.setSpacing(8)

        # Columna Izquierda: NVS y Configuración de Red/Token
        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)

        # 1. Bloque de Conexión Serial USB
        gb_conn = QGroupBox("Conexión Serial USB & Diagnóstico Directo")
        h_conn = QHBoxLayout(gb_conn)
        self.cb_ports = QComboBox()
        self.cb_ports.setToolTip("Seleccione el puerto serie COM asignado al ESP32 por USB.")
        self.btn_refresh = QPushButton("Refrescar")
        self.btn_connect = QPushButton("Conectar Serial")
        self.btn_disconnect = QPushButton("Desconectar")
        self.btn_disconnect.setEnabled(False)
        self.lbl_status = QLabel("DESCONECTADO")
        self.lbl_status.setStyleSheet("color: #ff7b72; font-weight: bold;")

        h_conn.addWidget(QLabel("COM:"))
        h_conn.addWidget(self.cb_ports, 1)
        h_conn.addWidget(self.btn_refresh)
        h_conn.addWidget(self.btn_connect)
        h_conn.addWidget(self.btn_disconnect)
        h_conn.addWidget(self.lbl_status)
        left_layout.addWidget(gb_conn)

        # 2. Bloque de Red Wi-Fi, IP Fija y Token de Seguridad NVS (V5)
        gb_wifi = QGroupBox("Credenciales Wi-Fi, IP Estática & Token de Sesión (NVS)")
        v_wifi = QVBoxLayout(gb_wifi)

        row_net = QHBoxLayout()
        self.cb_ssid = QComboBox()
        self.cb_ssid.setEditable(True)
        self.btn_scan = QPushButton("Escanear Redes")
        self.ed_pass = QLineEdit()
        self.ed_pass.setEchoMode(QLineEdit.Password)
        self.ed_devname = QLineEdit("CNC-XYZW-NETLOG")
        
        ip_box = QHBoxLayout()
        self.lbl_ip_prefix = QLabel("192.168.1.")
        self.lbl_ip_prefix.setStyleSheet("font-weight: bold; color: #58a6ff;")
        self.sp_ip4 = QSpinBox()
        self.sp_ip4.setRange(1, 254)
        self.sp_ip4.setValue(167)
        self.sp_ip4.setFixedWidth(60)
        ip_box.addWidget(self.lbl_ip_prefix)
        ip_box.addWidget(self.sp_ip4)

        row_net.addWidget(QLabel("SSID:"))
        row_net.addWidget(self.cb_ssid, 2)
        row_net.addWidget(self.btn_scan)
        row_net.addWidget(QLabel("Pass:"))
        row_net.addWidget(self.ed_pass, 2)
        row_net.addWidget(QLabel("Host:"))
        row_net.addWidget(self.ed_devname, 2)
        row_net.addWidget(QLabel("IP:"))
        row_net.addLayout(ip_box)
        v_wifi.addLayout(row_net)

        row_token_save = QHBoxLayout()
        self.ed_auth_token = QLineEdit()
        self.ed_auth_token.setPlaceholderText("Token TCP (vacío = abierto)")
        self.ed_auth_token.setToolTip("Token de sesión exigido por el firmware V5 para conexiones TCP.")
        self.btn_set_token = QPushButton("Fijar Token")
        self.btn_set_token.setToolTip("Envía SET_AUTH_TOKEN al ESP32 para guardarlo en NVS.")

        self.lbl_current = QLabel("Actual: ---")
        self.lbl_current.setStyleSheet("color: #7ee787; font-weight: bold;")
        self.btn_save_net = QPushButton("Guardar Red en NVS")
        self.btn_save_net.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")

        row_token_save.addWidget(QLabel("Token:"))
        row_token_save.addWidget(self.ed_auth_token, 1)
        row_token_save.addWidget(self.btn_set_token)
        row_token_save.addSpacing(10)
        row_token_save.addWidget(self.lbl_current, 1)
        row_token_save.addWidget(self.btn_save_net)
        v_wifi.addLayout(row_token_save)
        left_layout.addWidget(gb_wifi)

        # 3. Bloque Tabla NVS Ejes (12 Parámetros directos de V5)
        gb_nvs = QGroupBox("Parámetros Cinemáticos en NVS (Flash) - Pase cursor sobre títulos para ayuda")
        v_nvs = QVBoxLayout(gb_nvs)

        self.table_nvs = QTableWidget()
        self.table_nvs.setColumnCount(12)
        headers = [
            "Eje", "SPM", "Max Travel", "Backoff", "SoftOff",
            "Curva S", "Seek", "Feed", "BO us", "Man us", "Jog us", "Dir"
        ]
        self.table_nvs.setHorizontalHeaderLabels(headers)
        self.table_nvs.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        for col_idx, (title, desc) in NVS_HELP_DESCRIPTIONS.items():
            header_item = self.table_nvs.horizontalHeaderItem(col_idx)
            if header_item:
                header_item.setToolTip(f"<b>{title}</b><br>{desc}")

        v_nvs.addWidget(self.table_nvs, 1)

        row_btns = QHBoxLayout()
        self.btn_read = QPushButton("Leer NVS")
        self.btn_write = QPushButton("Guardar Fila Seleccionada")
        self.btn_write.setStyleSheet("background-color: #8957e5; color: white;")
        self.btn_restart = QPushButton("Reiniciar ESP32")
        self.btn_restart.setStyleSheet("background-color: #a42424; color: white;")

        row_btns.addWidget(self.btn_read)
        row_btns.addWidget(self.btn_write)
        row_btns.addStretch(1)
        row_btns.addWidget(self.btn_restart)
        v_nvs.addLayout(row_btns)
        left_layout.addWidget(gb_nvs, 1)

        # Columna Derecha: Panel de Prueba Libre sin Final de Carrera
        right_col = QGroupBox("Prueba de Movimiento Libre (Sin Final de Carrera)")
        right_col.setFixedWidth(350)
        right_layout = QVBoxLayout(right_col)
        right_layout.setContentsMargins(6, 6, 6, 6)
        right_layout.setSpacing(6)

        lbl_desc = QLabel(
            "<b>MODO FORZADO DIRECTO (USB):</b><br>"
            "Mueve los motores mediante ráfagas directas al hardware. "
            "<b>Ignora totalmente el estado de interruptores de final de carrera</b> y límites de software. "
            "Ideal para destrabar carros mecánicos o validar cableado con entradas flotantes."
        )
        lbl_desc.setWordWrap(True)
        lbl_desc.setStyleSheet("color: #ffb347; font-size: 11px;")
        right_layout.addWidget(lbl_desc)

        box_speed = QHBoxLayout()
        self.sp_jog_speed = QSpinBox()
        self.sp_jog_speed.setRange(80, 20000)
        self.sp_jog_speed.setValue(1000)
        self.sp_jog_speed.setSingleStep(100)
        self.sp_jog_speed.setToolTip("Periodo de paso en microsegundos (us) para el comando JOG_FREE (piso mínimo 80 us).")
        
        self.btn_enable_hw = QPushButton("Habilitar Actuadores")
        self.btn_enable_hw.setCheckable(True)
        self.btn_enable_hw.setStyleSheet("background-color: #1f6feb; color: white;")

        box_speed.addWidget(QLabel("Delay (us):"))
        box_speed.addWidget(self.sp_jog_speed)
        box_speed.addWidget(self.btn_enable_hw)
        right_layout.addLayout(box_speed)

        dp_container = QHBoxLayout()
        labels_xy = {"up": "Y+", "down": "Y-", "left": "X-", "right": "X+"}
        self.dpad_xy = DPadPainter(labels=labels_xy)
        self.dpad_zw = VerticalZWControl()

        dp_container.addWidget(self.dpad_xy, 3)
        dp_container.addWidget(self.dpad_zw, 2)
        right_layout.addLayout(dp_container, 1)

        self.lbl_jog_indicator = QLabel("ESTADO: EN ESPERA")
        self.lbl_jog_indicator.setAlignment(Qt.AlignCenter)
        self.lbl_jog_indicator.setStyleSheet("background-color: #0d1117; color: #58a6ff; font-weight: bold; border-radius: 4px; padding: 6px;")
        right_layout.addWidget(self.lbl_jog_indicator)

        main_h_layout.addWidget(left_col, 1)
        main_h_layout.addWidget(right_col)

        # Conexiones de Eventos
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect.clicked.connect(self._connect)
        self.btn_disconnect.clicked.connect(lambda: self.worker.stop() if self.worker else None)
        self.btn_scan.clicked.connect(lambda: self.worker.send_line("SCAN_WIFI") if self.worker else None)
        self.btn_save_net.clicked.connect(self._confirm_and_save_net)
        self.btn_set_token.clicked.connect(self._on_set_token_clicked)
        self.btn_read.clicked.connect(lambda: self.worker.send_line("DUMP_NVS") if self.worker else None)
        self.btn_write.clicked.connect(self._confirm_and_save_axis)
        self.btn_restart.clicked.connect(lambda: self.worker.send_line("RESTART_ESP") if self.worker else None)
        self.btn_enable_hw.toggled.connect(self._toggle_actuators)

        # Conexiones D-Pad XY (Pulsos sostenidos libres)
        self.dpad_xy.directionPressed.connect(lambda d: self._start_free_jog("xy", d))
        self.dpad_xy.directionReleased.connect(self._stop_free_jog)
        self.dpad_xy.stopPressed.connect(self._emergency_stop)

        # Conexiones Control ZW
        self.dpad_zw.directionPressed.connect(self._start_free_zw_jog)
        self.dpad_zw.directionReleased.connect(self._stop_free_jog)

    def _refresh_ports(self):
        self.cb_ports.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_ports.addItem(f"{p.device} ({p.description})", p.device)

    def _connect(self):
        port = self.cb_ports.currentData()
        if not port:
            QMessageBox.warning(self, "Puerto", "Seleccione un puerto COM válido.")
            return

        self.worker = SerialWorker()
        self.worker.configure(port, 115200)
        self.worker.connected.connect(lambda: (
            self.btn_connect.setEnabled(False),
            self.btn_disconnect.setEnabled(True),
            self.lbl_status.setText("CONECTADO"),
            self.lbl_status.setStyleSheet("color: #3fb950; font-weight: bold;"),
            QTimer.singleShot(300, lambda: self.worker.send_line("DUMP_NVS"))
        ))
        self.worker.disconnected.connect(lambda: (
            self.btn_connect.setEnabled(True),
            self.btn_disconnect.setEnabled(False),
            self.lbl_status.setText("DESCONECTADO"),
            self.lbl_status.setStyleSheet("color: #ff7b72; font-weight: bold;")
        ))
        self.worker.lineReceived.connect(self._on_line)
        self.worker.start()

    def _toggle_actuators(self, enabled):
        if self.worker:
            self.worker.send_line("CMD|enable_actuators" if enabled else "CMD|emergency_stop")
        self.btn_enable_hw.setText("Actuadores ON" if enabled else "Habilitar Actuadores")
        self.btn_enable_hw.setStyleSheet("background-color: #238636; color: white;" if enabled else "background-color: #1f6feb; color: white;")

    def _start_free_jog(self, group, d):
        if not self.btn_enable_hw.isChecked():
            self.lbl_jog_indicator.setText("AVISO: Active los actuadores")
            self.lbl_jog_indicator.setStyleSheet("background-color: #551a1a; color: #ff7b72; font-weight: bold; padding: 6px;")
            return

        axis = "y" if d in ("up", "down") else "x"
        direction = "forward" if d in ("up", "right") else "backward"
        self._current_jog_cmd = f"JOG_FREE|{axis}|{direction}|{self.sp_jog_speed.value()}"
        self.lbl_jog_indicator.setText(f"FORZANDO: {axis.upper()} {direction.upper()}")
        self.lbl_jog_indicator.setStyleSheet("background-color: #12303b; color: #00f0ff; font-weight: bold; padding: 6px;")
        self._send_jog_heartbeat()
        self.jog_timer.start()

    def _start_free_zw_jog(self, action):
        if not self.btn_enable_hw.isChecked():
            self.lbl_jog_indicator.setText("AVISO: Active los actuadores")
            self.lbl_jog_indicator.setStyleSheet("background-color: #551a1a; color: #ff7b72; font-weight: bold; padding: 6px;")
            return

        mapping = {
            "z_up": ("z", "forward"), "z_down": ("z", "backward"),
            "w_up": ("w", "forward"), "w_down": ("w", "backward")
        }
        if action in mapping:
            axis, direction = mapping[action]
            self._current_jog_cmd = f"JOG_FREE|{axis}|{direction}|{self.sp_jog_speed.value()}"
            self.lbl_jog_indicator.setText(f"FORZANDO: {axis.upper()} {direction.upper()}")
            self.lbl_jog_indicator.setStyleSheet("background-color: #12303b; color: #00f0ff; font-weight: bold; padding: 6px;")
            self._send_jog_heartbeat()
            self.jog_timer.start()

    def _send_jog_heartbeat(self):
        if self.worker and self._current_jog_cmd:
            self.worker.send_line(self._current_jog_cmd)

    def _stop_free_jog(self):
        self.jog_timer.stop()
        self._current_jog_cmd = None
        if self.worker:
            self.worker.send_line("JOG_STOP")
        self.lbl_jog_indicator.setText("ESTADO: EN ESPERA")
        self.lbl_jog_indicator.setStyleSheet("background-color: #0d1117; color: #58a6ff; font-weight: bold; padding: 6px;")

    def _emergency_stop(self):
        self.jog_timer.stop()
        self._current_jog_cmd = None
        if self.worker:
            self.worker.send_line("CMD|emergency_stop")
        self.btn_enable_hw.setChecked(False)
        self.lbl_jog_indicator.setText("PARADA DE EMERGENCIA")
        self.lbl_jog_indicator.setStyleSheet("background-color: #551a1a; color: #ff7b72; font-weight: bold; padding: 6px;")

    def _confirm_and_save_net(self):
        ssid = self.cb_ssid.currentText().strip()
        pwd = self.ed_pass.text().strip()
        devname = self.ed_devname.text().strip()
        ip4 = self.sp_ip4.value()

        if not ssid:
            QMessageBox.warning(self, "Validación", "El campo SSID no puede estar vacío.")
            return

        msg = f"¿Guardar configuración Wi-Fi en NVS?\n\n• SSID: {ssid}\n• Hostname: {devname}\n• IP Fija: 192.168.1.{ip4}"
        if QMessageBox.warning(self, "Confirmar Red", msg, QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            self.worker.send_line(f"SET_WIFI_NVS|{ssid}|{pwd}|{devname}|{ip4}")

    def _on_set_token_clicked(self):
        token = self.ed_auth_token.text().strip()
        if not self.worker:
            QMessageBox.warning(self, "Token", "Conéctese por puerto serie primero.")
            return
        self.worker.send_line(f"SET_AUTH_TOKEN|{token}")

    def _confirm_and_save_axis(self):
        row = self.table_nvs.currentRow()
        if row < 0:
            QMessageBox.warning(self, "NVS", "Seleccione una fila en la tabla de ejes.")
            return

        ax = self.table_nvs.item(row, 0).text().lower()
        # En V5, LOAD_AXIS_PARAM espera: ax|spm|max|bo|so|sc|seek|feed|bo_us|man|jog|dfl
        vals = [self.table_nvs.item(row, i).text() for i in range(1, 12)]
        if QMessageBox.warning(self, "Confirmar Eje", f"¿Sobrescribir parámetros del eje {ax.upper()} en NVS?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            self.worker.send_line(f"LOAD_AXIS_PARAM|{ax}|" + "|".join(vals))

    def _on_line(self, line: str):
        if line.startswith("WIFI_LIST|"):
            self.cb_ssid.clear()
            for s in line.split("|")[1].split(","):
                if s.strip():
                    self.cb_ssid.addItem(s.strip())
        elif line.startswith("ACK|SET_WIFI_NVS|OK"):
            if QMessageBox.question(self, "Reiniciar", "Red guardada con éxito en NVS.\n\n¿Desea reiniciar el ESP32 ahora?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.worker.send_line("RESTART_ESP")
        elif line.startswith("ACK|SET_AUTH_TOKEN|OK"):
            QMessageBox.information(self, "Token NVS", "Token de autenticación guardado en NVS exitosamente.")
        elif line.startswith("ACK|LOAD_AXIS_PARAM|OK"):
            QMessageBox.information(self, "NVS", "Parámetros de eje actualizados correctamente.")
        elif line.startswith("NVS_DATA|"):
            try:
                data = json.loads(line.split("|", 1)[1])
                self.cb_ssid.setCurrentText(data.get("ssid", ""))
                self.ed_devname.setText(data.get("devname", "CNC-XYZW-NETLOG"))
                ip4 = data.get("ip_oct4", 167)
                self.sp_ip4.setValue(ip4)
                self.lbl_current.setText(f"NVS IP: {data.get('current_ip')} | SSID: {data.get('ssid')} | IP Fija: 192.168.1.{ip4}")
                
                axes = data.get("axes", {})
                self.table_nvs.setRowCount(len(axes))
                for r, (k, v) in enumerate(axes.items()):
                    row_v = [
                        k.upper(), v.get("spm"), v.get("max"), v.get("bo"), v.get("so"),
                        v.get("sc"), v.get("seek"), v.get("feed"), v.get("bo_us"),
                        v.get("man_us"), v.get("jog_us"), v.get("dfl")
                    ]
                    for c, val in enumerate(row_v):
                        item = QTableWidgetItem(str(val))
                        title, desc = NVS_HELP_DESCRIPTIONS.get(c, ("", ""))
                        item.setToolTip(f"<b>{title} (Eje {k.upper()}):</b><br>{desc}")
                        if c == 0:
                            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                            item.setTextAlignment(Qt.AlignCenter)
                            item.setForeground(QColor("#ffb347"))
                        self.table_nvs.setItem(r, c, item)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Fallo al procesar NVS: {e}")

    def closeEvent(self, event):
        self.jog_timer.stop()
        if self.worker:
            self.worker.send_line("JOG_STOP")
            self.worker.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MaintenanceWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()