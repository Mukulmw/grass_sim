"""
Joint slider panel in robot units (J1/J2 in degrees, Z in millimetres), replacing
joint_state_publisher_gui, which can only show the raw ROS values (radians and metres).

Ranges are the software limits from robot.yaml. Each joint has a slider for dragging and a spin
box for typing an exact value; both edit the same value. The panel only holds the pose; the owner
converts it to radians/metres when publishing /joint_states.
"""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (QApplication, QDoubleSpinBox, QGridLayout, QHBoxLayout,
                                         QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)

DECIMALS = 1
SCALE = 10 ** DECIMALS          # slider ticks per unit (0.1 deg / 0.1 mm resolution)


class JointSliders(QWidget):

    JOINTS = (('j1', 'J1', 'deg'), ('z', 'Z', 'mm'), ('j2', 'J2', 'deg'))

    def __init__(self, cfg, font_pt, on_change):
        super().__init__()
        self._on_change = on_change
        self._rows = {}
        self.setWindowTitle('GRASS joints')
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        font = QFont('Sans', font_pt)
        self.setFont(font)

        grid = QGridLayout()
        for r, (key, label, unit) in enumerate(self.JOINTS):
            rng = getattr(cfg, key)
            name = QLabel(f'{label} [{unit}]')
            name.setFont(QFont('Sans', font_pt, QFont.Bold))
            slider = QSlider(Qt.Horizontal)
            slider.setRange(round(rng.minimum * SCALE), round(rng.maximum * SCALE))
            slider.setMinimumWidth(320)
            box = QDoubleSpinBox()
            box.setRange(rng.minimum, rng.maximum)
            box.setDecimals(DECIMALS)
            box.setSingleStep(1.0)
            box.setSuffix(f' {unit}')
            box.setAlignment(Qt.AlignRight)
            box.setKeyboardTracking(False)      # apply typed values on Enter / focus out
            limits = QLabel(f'{rng.minimum:g} … {rng.maximum:g}')
            limits.setStyleSheet('color: gray;')
            slider.valueChanged.connect(lambda v, b=box: b.setValue(v / SCALE))
            box.valueChanged.connect(lambda v, k=key: self._changed(k, v))
            grid.addWidget(name, r, 0)
            grid.addWidget(slider, r, 1)
            grid.addWidget(box, r, 2)
            grid.addWidget(limits, r, 3)
            self._rows[key] = (slider, box)

        buttons = QHBoxLayout()
        poses = (('Zero', (0.0, 0.0, 0.0)),
                 ('Home', _pose(cfg.home)),
                 ('Park', _pose(cfg.park)))
        for text, pose in poses:
            b = QPushButton(text)
            if pose is None:
                b.setEnabled(False)
                b.setToolTip(f'{text.lower()} pose is not configured in robot.yaml')
            else:
                b.setToolTip('j1 {:g} deg, z {:g} mm, j2 {:g} deg'.format(*pose))
                b.clicked.connect(lambda _=False, p=pose: self.set_pose(*p))
            buttons.addWidget(b)

        layout = QVBoxLayout()
        layout.addLayout(grid)
        layout.addLayout(buttons)
        self.setLayout(layout)
        self.set_pose(0.0, 0.0, 0.0)    # clamped into the limits by the spin boxes

    def _changed(self, key, value):
        slider = self._rows[key][0]
        tick = round(value * SCALE)
        if slider.value() != tick:
            slider.blockSignals(True)   # don't round-trip back into the spin box
            slider.setValue(tick)
            slider.blockSignals(False)
        self._on_change(self.pose())

    def pose(self):
        """Return (j1_deg, z_mm, j2_deg)."""
        return tuple(self._rows[k][1].value() for k, _, _ in self.JOINTS)

    def set_pose(self, j1_deg, z_mm, j2_deg):
        for (key, _, _), v in zip(self.JOINTS, (j1_deg, z_mm, j2_deg)):
            self._rows[key][1].setValue(v)

    def closeEvent(self, event):
        QApplication.quit()
        super().closeEvent(event)


def _pose(target):
    return None if target is None else (target.j1_deg, target.z_mm, target.j2_deg)
