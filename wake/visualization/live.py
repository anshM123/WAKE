"""Nonblocking live 3-D room viewer backed by PySide6 and pyqtgraph."""
from __future__ import annotations

from queue import Empty, Full, Queue
from pathlib import Path
from typing import Any
import numpy as np

from wake.pose.transforms import quaternion_to_matrix


class LatestOnlyQueue:
    """Single-slot queue that discards stale visualization snapshots."""

    def __init__(self) -> None:
        self.queue: Queue = Queue(maxsize=1)

    def put(self, value: Any) -> None:
        try:
            self.queue.put_nowait(value)
        except Full:
            try:
                self.queue.get_nowait()
            except Empty:
                pass
            self.queue.put_nowait(value)

    def get(self, timeout: float | None = None) -> Any:
        return self.queue.get(timeout=timeout)

    def latest(self) -> Any | None:
        try:
            return self.queue.get_nowait()
        except Empty:
            return None


def voxel_display_arrays(
    voxels: tuple,
    voxel_size_m: float,
    occupancy_threshold: float = .55,
    confidence_threshold: float = .1,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert scientific voxels into filtered positions and grayscale RGBA."""
    positions, colors = [], []
    for index, occupancy, confidence, observations in voxels:
        if occupancy < occupancy_threshold or confidence < confidence_threshold:
            continue
        positions.append([(axis + .5) * voxel_size_m for axis in index])
        strength = min(1.0, max(0.0, (occupancy - .5) * 2))
        agreement = min(1.0, observations / 10.0) * confidence
        darkness = .85 - .7 * strength * agreement
        alpha = .08 + .82 * strength * agreement
        colors.append([darkness, darkness, darkness, alpha])
    if not positions:
        return np.empty((0, 3), dtype=float), np.empty((0, 4), dtype=float)
    return np.asarray(positions, dtype=float), np.asarray(colors, dtype=float)


class LiveMapViewer:
    """Interactive popup that consumes immutable snapshots at a bounded rate."""

    def __init__(self, snapshot_queue: LatestOnlyQueue, *, voxel_size_m: float, update_hz: float = 15.0, occupancy_threshold: float = .55, confidence_threshold: float = .1) -> None:
        try:
            from PySide6 import QtCore, QtWidgets
            import pyqtgraph.opengl as gl
        except ImportError as exc:
            raise RuntimeError("install wake-mapper[ui] for the live viewer") from exc
        self.QtCore, self.QtWidgets, self.gl = QtCore, QtWidgets, gl
        self.snapshot_queue = snapshot_queue
        self.voxel_size_m = voxel_size_m
        self.occupancy_threshold = occupancy_threshold
        self.confidence_threshold = confidence_threshold
        self.paused = False
        self.last_snapshot = None
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("WAKE LIVE — probabilistic room map")
        self.window.resize(1300, 850)
        self._build_ui()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(max(1, round(1000 / update_hz)))

    def _build_ui(self) -> None:
        central = self.QtWidgets.QWidget()
        layout = self.QtWidgets.QHBoxLayout(central)
        self.view = self.gl.GLViewWidget()
        self.view.setCameraPosition(distance=4, elevation=25, azimuth=35)
        self.view.opts["center"] = self._vector(0, 0, 1)
        layout.addWidget(self.view, stretch=4)
        side = self.QtWidgets.QVBoxLayout()
        self.status = self.QtWidgets.QLabel("WAKE LIVE\nWaiting for data…")
        self.status.setStyleSheet("font-family: Consolas; font-size: 13px;")
        self.status.setAlignment(self.QtCore.Qt.AlignTop)
        side.addWidget(self.status, stretch=1)
        for label, callback in (
            ("Reset camera", self._reset_camera), ("Top view", self._top_view),
            ("Side view", self._side_view), ("Perspective", self._perspective),
            ("Pause / resume", self._toggle_pause), ("Save snapshot", self._save_snapshot),
        ):
            button = self.QtWidgets.QPushButton(label); button.clicked.connect(callback); side.addWidget(button)
        self.voxel_toggle = self.QtWidgets.QCheckBox("Voxels"); self.voxel_toggle.setChecked(True); side.addWidget(self.voxel_toggle)
        self.trajectory_toggle = self.QtWidgets.QCheckBox("Trajectory"); self.trajectory_toggle.setChecked(True); side.addWidget(self.trajectory_toggle)
        self.plane_toggle = self.QtWidgets.QCheckBox("Fitted planes"); self.plane_toggle.setChecked(True); side.addWidget(self.plane_toggle)
        layout.addLayout(side, stretch=1)
        self.window.setCentralWidget(central)
        grid = self.gl.GLGridItem(); grid.setSize(6, 6); grid.setSpacing(.25, .25); self.view.addItem(grid)
        self.voxel_item = self.gl.GLScatterPlotItem(size=6, pxMode=True); self.view.addItem(self.voxel_item)
        self.trajectory_item = self.gl.GLLinePlotItem(color=(.1, .4, 1, .8), width=2, antialias=True); self.view.addItem(self.trajectory_item)
        self.drone_item = self.gl.GLScatterPlotItem(size=13, color=(1, .25, .1, 1), pxMode=True); self.view.addItem(self.drone_item)
        self.heading_item = self.gl.GLLinePlotItem(color=(1, .2, .1, 1), width=3); self.view.addItem(self.heading_item)
        self.surface_item = self.gl.GLLinePlotItem(color=(1, .75, .1, .9), width=3); self.view.addItem(self.surface_item)

    def _vector(self, x: float, y: float, z: float):
        from PySide6.QtGui import QVector3D
        return QVector3D(x, y, z)

    def run(self) -> int:
        self.window.show()
        return self.app.exec()

    def _update(self) -> None:
        if self.paused:
            return
        snapshot = self.snapshot_queue.latest()
        if snapshot is None:
            return
        self.last_snapshot = snapshot
        positions, colors = voxel_display_arrays(snapshot.voxels, self.voxel_size_m, self.occupancy_threshold, self.confidence_threshold)
        self.voxel_item.setData(pos=positions, color=colors, size=6)
        self.voxel_item.setVisible(self.voxel_toggle.isChecked())
        trajectory = np.asarray(snapshot.trajectory, dtype=float)
        self.trajectory_item.setData(pos=trajectory if len(trajectory) else np.empty((0, 3)))
        self.trajectory_item.setVisible(self.trajectory_toggle.isChecked())
        if snapshot.current_position is not None:
            position = np.asarray(snapshot.current_position, dtype=float)
            self.drone_item.setData(pos=position.reshape(1, 3))
            rotation = quaternion_to_matrix(snapshot.current_rotation)
            heading = position + rotation @ np.asarray([.25, 0, 0])
            self.heading_item.setData(pos=np.vstack([position, heading]))
            if snapshot.surface is not None:
                endpoint = position + rotation @ np.asarray(snapshot.surface.normal_body) * snapshot.surface.distance_m
                self.surface_item.setData(pos=np.vstack([position, endpoint]))
            else:
                self.surface_item.setData(pos=np.empty((0, 3)))
        self.status.setText(self._status_text(snapshot))

    def _status_text(self, snapshot) -> str:
        h, s = snapshot.health, snapshot.surface
        surface = "none" if s is None else f"{s.nearby_probability:.2f}\nRange: {s.distance_m:.2f} ± {s.distance_sigma_m:.2f} m"
        tag = "GOOD" if h.tag_visible else "LOST"
        failures = ", ".join(h.failure_modes) if h.failure_modes else "none"
        return f"""WAKE LIVE

Telemetry: {h.telemetry_hz:.1f} Hz
IMU: {h.imu_hz:.1f} Hz
Pose: {h.pose_hz:.1f} Hz

Clock RTT: {h.clock_sync_rtt_ms:.2f} ms
Sync error: {h.sync_error_ms:.2f} ms

Tag: {tag}
Reprojection: {h.reprojection_error_px:.2f} px

Surface probability: {surface}

Map voxels: {len(snapshot.voxels)}
State: MANUAL_MAPPING
Safety: {'HOLD' if failures != 'none' else 'MONITORING'}

Battery: {h.battery_v:.2f} V
Failures: {failures}"""

    def _reset_camera(self) -> None: self.view.setCameraPosition(distance=4, elevation=25, azimuth=35)
    def _top_view(self) -> None: self.view.setCameraPosition(distance=4, elevation=90, azimuth=0)
    def _side_view(self) -> None: self.view.setCameraPosition(distance=4, elevation=0, azimuth=0)
    def _perspective(self) -> None: self.view.setCameraPosition(distance=4, elevation=25, azimuth=35)
    def _toggle_pause(self) -> None: self.paused = not self.paused

    def _save_snapshot(self) -> None:
        path, _ = self.QtWidgets.QFileDialog.getSaveFileName(self.window, "Save WAKE view", str(Path.cwd() / "wake_snapshot.png"), "PNG image (*.png)")
        if path:
            self.view.grabFramebuffer().save(path)
