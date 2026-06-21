import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import threading
import time
from enum import Enum
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QTextEdit, QFrame,
    QDialog, QLineEdit, QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor

# ─── Goal points ─────────────────────────────────────────────
GOALS = [
    {"name": "Point A", "x": 2.0,  "y": 1.0,   "z": 0.0,     "w": 1.0},
    {"name": "Point B", "x": 4.11, "y": 0.552,  "z": 0.00247, "w": 1.0},
    {"name": "Point C", "x": 2.07, "y": -1.68,  "z": 0.00647, "w": 1.0},
]

ROBOTS_CFG = [
    {"id": "TB1", "namespace": "", "color": "#378ADD"},
    {"id": "TB2", "namespace": "", "color": "#1D9E75"},
    # When namespaced:
    # {"id": "TB1", "namespace": "/tb1", "color": "#378ADD"},
    # {"id": "TB2", "namespace": "/tb2", "color": "#1D9E75"},
]

class RobotState(Enum):
    IDLE       = "Idle"
    NAVIGATING = "Navigating"
    REACHED    = "Reached"
    ERROR      = "Error"


# ─── Signals (cross-thread UI updates) ───────────────────────
class Signals(QObject):
    refresh = pyqtSignal()


# ─── Robot Agent ─────────────────────────────────────────────
class RobotAgent:
    def __init__(self, node, cfg):
        self.node         = node
        self.id           = cfg["id"]
        self.color        = cfg["color"]
        ns                = cfg["namespace"]
        topic             = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        self._client      = ActionClient(node, NavigateToPose, topic)
        self._handle      = None
        self.state        = RobotState.IDLE
        self.current_goal = None
        self.distance_remaining = 0.0
        self.tasks_done   = 0
        self.on_reached   = None
        self.on_failed    = None

    def is_free(self):
        return self.state == RobotState.IDLE

    def send_goal(self, goal):
        self.state        = RobotState.NAVIGATING
        self.current_goal = goal

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id    = "map"
        msg.pose.header.stamp       = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x    = goal["x"]
        msg.pose.pose.position.y    = goal["y"]
        msg.pose.pose.position.z    = goal["z"]
        msg.pose.pose.orientation.w = goal["w"]

        future = self._client.send_goal_async(
            msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def cancel(self):
        if self._handle:
            self._handle.cancel_goal_async()
        self.state        = RobotState.IDLE
        self.current_goal = None

    def _goal_response_cb(self, future):
        self._handle = future.result()
        if not self._handle.accepted:
            self.state = RobotState.ERROR
            if self.on_failed:
                self.on_failed(self)
            return
        self._handle.get_result_async().add_done_callback(
            self._result_cb)

    def _feedback_cb(self, msg):
        self.distance_remaining = msg.feedback.distance_remaining

    def _result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.state       = RobotState.REACHED
            self.tasks_done += 1
            if self.on_reached:
                self.on_reached(self)
        else:
            self.state = RobotState.ERROR
            if self.on_failed:
                self.on_failed(self)


# ─── Task Queue ──────────────────────────────────────────────
class TaskQueue:
    def __init__(self):
        self._q    = deque()
        self._lock = threading.Lock()
        self._log  = []

    def add(self, goal):
        with self._lock:
            self._q.append(goal)
        self._log_msg(f"Queued → {goal['name']}")

    def pop(self):
        with self._lock:
            return self._q.popleft() if self._q else None

    def peek_all(self):
        with self._lock:
            return list(self._q)

    def clear(self):
        with self._lock:
            self._q.clear()

    def _log_msg(self, msg):
        self._log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        if len(self._log) > 100:
            self._log = self._log[-100:]

    def log(self, msg):
        self._log_msg(msg)

    def get_log(self):
        return list(reversed(self._log))


# ─── Fleet Manager ───────────────────────────────────────────
class FleetManager:
    def __init__(self, node, signals):
        self.node    = node
        self.signals = signals
        self.robots  = [RobotAgent(node, cfg) for cfg in ROBOTS_CFG]
        self.queue   = TaskQueue()
        self._lock   = threading.Lock()

        for r in self.robots:
            r.on_reached = self._on_reached
            r.on_failed  = self._on_failed

    def add_task(self, goal):
        self.queue.add(goal)
        self._try_dispatch()
        self.signals.refresh.emit()

    def send_to_robot(self, robot_id, goal):
        r = self._get(robot_id)
        if not r:
            return
        if r.state == RobotState.NAVIGATING:
            r.cancel()
        r.send_goal(goal)
        self.queue.log(f"Manual: {r.id} → {goal['name']}")
        self.signals.refresh.emit()

    def cancel_robot(self, robot_id):
        r = self._get(robot_id)
        if r:
            r.cancel()
            self.queue.log(f"{r.id} cancelled")
            self.signals.refresh.emit()

    def cancel_all(self):
        for r in self.robots:
            r.cancel()
        self.queue.clear()
        self.queue.log("All tasks cancelled")
        self.signals.refresh.emit()

    def wait_for_servers(self):
        for r in self.robots:
            r._client.wait_for_server()
            self.queue.log(f"{r.id} connected to Nav2 ✓")
            self.signals.refresh.emit()

    def _try_dispatch(self):
        with self._lock:
            for r in self.robots:
                if r.is_free():
                    task = self.queue.pop()
                    if task:
                        r.send_goal(task)
                        self.queue.log(
                            f"Auto-assigned {task['name']} → {r.id}")

    def _on_reached(self, robot):
        robot.state = RobotState.IDLE
        self.queue.log(f"{robot.id} reached goal ✓  "
                       f"(total: {robot.tasks_done})")
        self._try_dispatch()
        self.signals.refresh.emit()

    def _on_failed(self, robot):
        self.queue.log(f"{robot.id} failed — requeueing task")
        if robot.current_goal:
            self.queue.add(robot.current_goal)
        robot.state        = RobotState.IDLE
        robot.current_goal = None
        self._try_dispatch()
        self.signals.refresh.emit()

    def _get(self, robot_id):
        return next((r for r in self.robots if r.id == robot_id), None)


# ─── Add Goal Dialog ─────────────────────────────────────────
class AddGoalDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add goal point")
        self.setFixedSize(280, 200)
        self.result_goal = None

        layout = QFormLayout(self)
        self.f_name = QLineEdit("Point D")
        self.f_x    = QLineEdit("0.0")
        self.f_y    = QLineEdit("0.0")
        self.f_z    = QLineEdit("0.0")
        layout.addRow("Name", self.f_name)
        layout.addRow("X",    self.f_x)
        layout.addRow("Y",    self.f_y)
        layout.addRow("Z",    self.f_z)

        btn = QPushButton("Add goal")
        btn.setStyleSheet(
            "background:#378ADD;color:white;padding:8px;"
            "border:none;border-radius:4px;")
        btn.clicked.connect(self._confirm)
        layout.addRow(btn)

    def _confirm(self):
        try:
            self.result_goal = {
                "name": self.f_name.text().strip(),
                "x":    float(self.f_x.text()),
                "y":    float(self.f_y.text()),
                "z":    float(self.f_z.text()),
                "w":    1.0,
            }
            self.accept()
        except ValueError:
            QMessageBox.warning(self, "Error", "X, Y, Z must be numbers")


# ─── Robot Panel Widget ──────────────────────────────────────
class RobotPanel(QFrame):
    def __init__(self, robot, fleet, parent=None):
        super().__init__(parent)
        self.robot = robot
        self.fleet = fleet
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(
            "QFrame{background:white;border:1px solid #ddd;"
            "border-radius:6px;}")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)

        # Top row
        top = QHBoxLayout()
        self.dot = QLabel("●")
        self.dot.setStyleSheet(f"color:{robot.color};font-size:18px;")
        self.name_lbl = QLabel(robot.id)
        self.name_lbl.setFont(QFont("Helvetica", 12, QFont.Bold))
        self.state_lbl = QLabel("● Idle")
        self.state_lbl.setStyleSheet(
            "background:#eee;color:#888;padding:2px 8px;"
            "border-radius:4px;font-size:11px;")
        self.dist_lbl = QLabel("")
        self.dist_lbl.setStyleSheet("color:#aaa;font-size:10px;")
        top.addWidget(self.dot)
        top.addWidget(self.name_lbl)
        top.addWidget(self.state_lbl)
        top.addStretch()
        top.addWidget(self.dist_lbl)
        layout.addLayout(top)

        # Goal label
        self.goal_lbl = QLabel("No task")
        self.goal_lbl.setStyleSheet("color:#888;font-size:11px;")
        layout.addWidget(self.goal_lbl)

        # Tasks done + cancel
        bot = QHBoxLayout()
        self.done_lbl = QLabel("Tasks done: 0")
        self.done_lbl.setStyleSheet("color:#aaa;font-size:10px;")
        cancel_btn = QPushButton("Cancel")
        cancel_btn.setStyleSheet(
            "color:#E24B4A;background:white;border:1px solid #E24B4A;"
            "border-radius:4px;padding:3px 10px;font-size:10px;")
        cancel_btn.setCursor(Qt.PointingHandCursor)
        cancel_btn.clicked.connect(
            lambda: fleet.cancel_robot(robot.id))
        bot.addWidget(self.done_lbl)
        bot.addStretch()
        bot.addWidget(cancel_btn)
        layout.addLayout(bot)

    def refresh(self):
        r = self.robot
        state_styles = {
            RobotState.IDLE:       ("● Idle",       "#888888"),
            RobotState.NAVIGATING: ("◉ Navigating", "#378ADD"),
            RobotState.REACHED:    ("✓ Reached",    "#1D9E75"),
            RobotState.ERROR:      ("✕ Error",       "#E24B4A"),
        }
        text, color = state_styles[r.state]
        self.state_lbl.setText(text)
        self.state_lbl.setStyleSheet(
            f"color:{color};background:#f5f5f5;padding:2px 8px;"
            f"border-radius:4px;font-size:11px;font-weight:bold;")

        if r.current_goal:
            self.goal_lbl.setText(f"→ {r.current_goal['name']}  "
                                   f"({r.current_goal['x']}, "
                                   f"{r.current_goal['y']})")
            self.goal_lbl.setStyleSheet("color:#111;font-size:11px;")
        else:
            self.goal_lbl.setText("No task")
            self.goal_lbl.setStyleSheet("color:#888;font-size:11px;")

        if r.state == RobotState.NAVIGATING:
            self.dist_lbl.setText(f"{r.distance_remaining:.1f} m away")
        else:
            self.dist_lbl.setText("")

        self.done_lbl.setText(f"Tasks done: {r.tasks_done}")


# ─── Main Window ─────────────────────────────────────────────
class FleetWindow(QMainWindow):
    def __init__(self, fleet, goals):
        super().__init__()
        self.fleet = fleet
        self.goals = goals
        self.setWindowTitle("AMR Fleet Dashboard")
        self.setMinimumSize(760, 620)

        fleet.signals.refresh.connect(self._refresh)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Header ──
        hdr = QWidget()
        hdr.setStyleSheet("background:#111;")
        hdr.setFixedHeight(50)
        hdr_l = QHBoxLayout(hdr)
        hdr_l.setContentsMargins(16, 0, 16, 0)
        title = QLabel("AMR Fleet Dashboard")
        title.setStyleSheet(
            "color:white;font-size:16px;font-weight:bold;")
        sub = QLabel("2 × TurtleBot3")
        sub.setStyleSheet("color:#666;font-size:11px;")
        cancel_all = QPushButton("⬛  Cancel all")
        cancel_all.setStyleSheet(
            "background:#E24B4A;color:white;border:none;"
            "border-radius:4px;padding:6px 14px;font-size:11px;")
        cancel_all.setCursor(Qt.PointingHandCursor)
        cancel_all.clicked.connect(fleet.cancel_all)
        hdr_l.addWidget(title)
        hdr_l.addWidget(sub)
        hdr_l.addStretch()
        hdr_l.addWidget(cancel_all)
        root.addWidget(hdr)

        # ── Body ──
        body = QWidget()
        body.setStyleSheet("background:#f5f5f5;")
        body_l = QHBoxLayout(body)
        body_l.setContentsMargins(12, 12, 12, 12)
        body_l.setSpacing(12)
        root.addWidget(body)

        # Left column
        left = QVBoxLayout()
        left.setSpacing(8)

        # Robot panels
        lbl = QLabel("ROBOTS")
        lbl.setStyleSheet("color:#aaa;font-size:10px;font-weight:bold;")
        left.addWidget(lbl)

        self._robot_panels = {}
        for robot in fleet.robots:
            panel = RobotPanel(robot, fleet)
            self._robot_panels[robot.id] = panel
            left.addWidget(panel)

        # Queue
        ql = QLabel("TASK QUEUE")
        ql.setStyleSheet("color:#aaa;font-size:10px;font-weight:bold;")
        left.addWidget(ql)

        self._queue_list = QListWidget()
        self._queue_list.setMaximumHeight(110)
        self._queue_list.setStyleSheet(
            "background:white;border:1px solid #ddd;"
            "border-radius:4px;font-size:11px;font-family:monospace;")
        left.addWidget(self._queue_list)

        # Log
        ll = QLabel("LOG")
        ll.setStyleSheet("color:#aaa;font-size:10px;font-weight:bold;")
        left.addWidget(ll)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setStyleSheet(
            "background:white;border:1px solid #ddd;"
            "border-radius:4px;font-size:10px;font-family:monospace;")
        left.addWidget(self._log)

        body_l.addLayout(left, stretch=3)

        # Right column — goals
        right = QVBoxLayout()
        right.setSpacing(6)
        rl = QLabel("GOAL POINTS")
        rl.setStyleSheet("color:#aaa;font-size:10px;font-weight:bold;")
        right.addWidget(rl)

        self._goals_layout = QVBoxLayout()
        self._goals_layout.setSpacing(4)
        right.addLayout(self._goals_layout)
        self._build_goal_buttons()

        add_btn = QPushButton("＋  Add goal")
        add_btn.setStyleSheet(
            "background:white;color:#378ADD;border:1px solid #378ADD;"
            "border-radius:4px;padding:8px;font-size:11px;")
        add_btn.setCursor(Qt.PointingHandCursor)
        add_btn.clicked.connect(self._add_goal)
        right.addWidget(add_btn)
        right.addStretch()

        body_l.addLayout(right, stretch=2)

        # Poll timer for live distance updates
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(500)

    def _build_goal_buttons(self):
        # Clear
        while self._goals_layout.count():
            item = self._goals_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        for goal in self.goals:
            g = goal
            card = QFrame()
            card.setStyleSheet(
                "QFrame{background:white;border:1px solid #ddd;"
                "border-radius:6px;}")
            cl = QVBoxLayout(card)
            cl.setContentsMargins(8, 6, 8, 6)
            cl.setSpacing(4)

            name = QLabel(g["name"])
            name.setStyleSheet("font-weight:bold;font-size:12px;")
            coord = QLabel(f"x={g['x']}  y={g['y']}")
            coord.setStyleSheet(
                "color:#aaa;font-size:10px;font-family:monospace;")
            cl.addWidget(name)
            cl.addWidget(coord)

            btns = QHBoxLayout()
            btns.setSpacing(4)

            # Queue button
            q_btn = QPushButton("Queue")
            q_btn.setStyleSheet(
                "background:#378ADD;color:white;border:none;"
                "border-radius:4px;padding:5px;font-size:10px;")
            q_btn.setCursor(Qt.PointingHandCursor)
            q_btn.clicked.connect(
                lambda _, g=g: self.fleet.add_task(g))
            btns.addWidget(q_btn)

            # Per-robot direct buttons
            for robot in self.fleet.robots:
                r = robot
                rb = QPushButton(r.id)
                rb.setStyleSheet(
                    f"background:{r.color};color:white;border:none;"
                    f"border-radius:4px;padding:5px;font-size:10px;")
                rb.setCursor(Qt.PointingHandCursor)
                rb.clicked.connect(
                    lambda _, g=g, rid=r.id:
                        self.fleet.send_to_robot(rid, g))
                btns.addWidget(rb)

            cl.addLayout(btns)
            self._goals_layout.addWidget(card)

    def _add_goal(self):
        dlg = AddGoalDialog(self)
        if dlg.exec_() == QDialog.Accepted and dlg.result_goal:
            self.goals.append(dlg.result_goal)
            self.fleet.queue.log(
                f"Goal added: {dlg.result_goal['name']}")
            self._build_goal_buttons()
            self._refresh()

    def _refresh(self):
        for panel in self._robot_panels.values():
            panel.refresh()

        self._queue_list.clear()
        for i, task in enumerate(self.fleet.queue.peek_all()):
            self._queue_list.addItem(
                f"  {i+1}.  {task['name']}  "
                f"({task['x']}, {task['y']})")

        self._log.setPlainText(
            "\n".join(self.fleet.queue.get_log()))


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = Node('fleet_manager')

    signals = Signals()
    fleet   = FleetManager(node, signals)

    threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True).start()

    threading.Thread(
        target=fleet.wait_for_servers, daemon=True).start()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = FleetWindow(fleet, GOALS)
    win.show()

    exit_code = app.exec_()

    fleet.cancel_all()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()