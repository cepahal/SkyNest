import sys
import os
import pybullet as p
import pybullet_data
import time
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QFileDialog, QTableWidget, QTableWidgetItem, QLabel, QHBoxLayout,
    QMessageBox, QDialog, QSlider, QFormLayout, QDoubleSpinBox, QGroupBox,
    QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, QProcess

# -------------------------
# Field Adjust Dialog
# -------------------------
class FieldAdjustDialog(QDialog):
    """
    Dialog that appears immediately after loading a field.
    Lets user rotate X/Y/Z in ±90° steps and set a uniform scale (percent).
    Live-updates the field preview by calling parent.apply_field_transform.
    """

    def __init__(self, parent, mesh_path, initial_scale=1.0, initial_euler=(0.0,0.0,0.0), base_position=(0,0,0.875)):
        super().__init__(parent)
        self.parent = parent
        self.mesh_path = mesh_path
        self.setWindowTitle("Adjust Field Orientation & Scale")
        self.resize(420, 260)

        self.euler = list(initial_euler)  # degrees [rx, ry, rz]
        self.scale = initial_scale
        self.base_position = base_position

        layout = QVBoxLayout(self)

        # Grid for rotation buttons and current angles
        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Rotate ±30°</b>"), 0, 0, 1, 3)

        # X rotation row
        grid.addWidget(QLabel(f"<span style='color:#3498db;'>X:</span>"), 1, 0)
        self.btn_x_neg = QPushButton("⟲ -30°")
        self.btn_x_pos = QPushButton("30° ⟳")
        self.lbl_x = QLabel(f"{self.euler[0]:.0f}°")
        grid.addWidget(self.btn_x_neg, 1, 1)
        grid.addWidget(self.btn_x_pos, 1, 2)
        grid.addWidget(self.lbl_x, 1, 3)

        # Y rotation row
        grid.addWidget(QLabel(f"<span style='color:#2ecc71;'>Y:</span>"), 2, 0)
        self.btn_y_neg = QPushButton("⟲ -30°")
        self.btn_y_pos = QPushButton("30° ⟳")
        self.lbl_y = QLabel(f"{self.euler[1]:.0f}°")
        grid.addWidget(self.btn_y_neg, 2, 1)
        grid.addWidget(self.btn_y_pos, 2, 2)
        grid.addWidget(self.lbl_y, 2, 3)

        # Z rotation row
        grid.addWidget(QLabel(f"<span style='color:#e74c3c;'>Z:</span>"), 3, 0)
        self.btn_z_neg = QPushButton("⟲ -30°")
        self.btn_z_pos = QPushButton("30° ⟳")
        self.lbl_z = QLabel(f"{self.euler[2]:.0f}°")
        grid.addWidget(self.btn_z_neg, 3, 1)
        grid.addWidget(self.btn_z_pos, 3, 2)
        grid.addWidget(self.lbl_z, 3, 3)

        layout.addLayout(grid)

        # Scale control (uniform)
        scale_box = QGroupBox("Uniform Scale (percent)")
        s_layout = QVBoxLayout()
        self.scale_spin = QDoubleSpinBox()
        self.scale_spin.setRange(.0001, 1000.0)   # 1% to 1000%
        self.scale_spin.setSuffix("%")
        self.scale_spin.setDecimals(1)
        self.scale_spin.setSingleStep(1.0)
        self.scale_spin.setValue(self.scale * 100.0)
        s_layout.addWidget(self.scale_spin)
        scale_box.setLayout(s_layout)
        layout.addWidget(scale_box)

        # Buttons
        btn_row = QHBoxLayout()
        self.apply_btn = QPushButton("Apply")
        self.reset_btn = QPushButton("Reset")
        self.done_btn = QPushButton("Done")
        btn_row.addWidget(self.apply_btn)
        btn_row.addWidget(self.reset_btn)
        btn_row.addWidget(self.done_btn)
        layout.addLayout(btn_row)

        # Connect signals
        self.btn_x_neg.clicked.connect(lambda: self.rotate_axis(0, -45))
        self.btn_x_pos.clicked.connect(lambda: self.rotate_axis(0, 45))
        self.btn_y_neg.clicked.connect(lambda: self.rotate_axis(1, -45))
        self.btn_y_pos.clicked.connect(lambda: self.rotate_axis(1, 45))
        self.btn_z_neg.clicked.connect(lambda: self.rotate_axis(2, -45))
        self.btn_z_pos.clicked.connect(lambda: self.rotate_axis(2, 45))

        self.scale_spin.valueChanged.connect(self.scale_changed)
        self.apply_btn.clicked.connect(self.apply_clicked)
        self.reset_btn.clicked.connect(self.reset_clicked)
        self.done_btn.clicked.connect(self.accept)

        # initial live preview
        self.apply_transform()

    def rotate_axis(self, idx, deg):
        self.euler[idx] = (self.euler[idx] + deg) % 360
        self.update_labels()
        # Live preview
        self.apply_transform()

    def update_labels(self):
        self.lbl_x.setText(f"{self.euler[0]:.0f}°")
        self.lbl_y.setText(f"{self.euler[1]:.0f}°")
        self.lbl_z.setText(f"{self.euler[2]:.0f}°")

    def scale_changed(self, val):
        self.scale = val / 100.0
        # live preview
        self.apply_transform()

    def apply_transform(self):
        """Apply the transform to the parent field (live preview)."""
        if hasattr(self.parent, "apply_field_transform"):
            self.parent.apply_field_transform(
                self.mesh_path, 
                self.scale, 
                tuple(self.euler), 
                base_position=self.base_position
            )

    def apply_clicked(self):
        # same as apply_transform but keep for UI clarity
        self.apply_transform()

    def reset_clicked(self):
        # Reset to defaults
        self.euler = [0.0, 0.0, 0.0]
        self.scale = 1.0
        self.scale_spin.setValue(100.0)
        self.update_labels()
        self.apply_transform()


# -------------------------
# Existing classes (GamePieceConfigDialog adjusted)
# -------------------------
class GamePieceConfigDialog(QDialog):
    def __init__(self, parent, robot_id, piece_id, mesh_path):
        super().__init__(parent)
        self.setWindowTitle("Configure Game Piece Position")
        self.resize(400, 550)
        self.robot_id = robot_id
        self.piece_id = piece_id
        self.mesh_path = mesh_path
        
        # Default Offsets
        self.intake_offset = {"pos": [0, 0, 0.5], "orn": [0, 0, 0]}
        self.outtake_offset = {"pos": [0, 0, 0.5], "orn": [0, 0, 0]}
        
        # Current values being edited
        self.current_pos = [0.0, 0.0, 0.5]
        self.current_rpy = [0.0, 0.0, 0.0]

        # Track scale (1.0 = 100%)
        self.current_scale = 1.0

        layout = QVBoxLayout()
        
        # --- Sliders Section ---
        controls_grp = QGroupBox("Adjust Relative Position")
        form = QFormLayout()
        
        self.spin_x = self.create_spinbox(-2.0, 2.0, 0.0)
        self.spin_y = self.create_spinbox(-2.0, 2.0, 0.0)
        self.spin_z = self.create_spinbox(-2.0, 2.0, 0.5)
        self.spin_roll = self.create_spinbox(-180, 180, 0)
        self.spin_pitch = self.create_spinbox(-180, 180, 0)
        self.spin_yaw = self.create_spinbox(-180, 180, 0)

        form.addRow("X Offset (m):", self.spin_x)
        form.addRow("Y Offset (m):", self.spin_y)
        form.addRow("Z Offset (m):", self.spin_z)
        form.addRow("Roll (deg):", self.spin_roll)
        form.addRow("Pitch (deg):", self.spin_pitch)
        form.addRow("Yaw (deg):", self.spin_yaw)

        controls_grp.setLayout(form)
        layout.addWidget(controls_grp)

        # --- SCALE SLIDER ---
        scale_grp = QGroupBox("Game Piece Scale")
        scale_layout = QVBoxLayout()

        self.scale_slider = QSlider(Qt.Horizontal)
        self.scale_slider.setMinimum(10)     # 10%
        self.scale_slider.setMaximum(300)    # 300%
        self.scale_slider.setValue(100)      # default 100%
        self.scale_slider.setTickInterval(10)
        self.scale_slider.setTickPosition(QSlider.TicksBelow)

        self.scale_label = QLabel("Scale: 100%")

        scale_layout.addWidget(self.scale_label)
        scale_layout.addWidget(self.scale_slider)
        scale_grp.setLayout(scale_layout)
        layout.addWidget(scale_grp)

        # --- Buttons ---
        btn_layout = QHBoxLayout()
        self.btn_save_intake = QPushButton("Save as INTAKE Pos")
        self.btn_save_outtake = QPushButton("Save as OUTTAKE Pos")

        self.btn_save_intake.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
        self.btn_save_outtake.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")

        self.btn_save_intake.clicked.connect(self.save_intake)
        self.btn_save_outtake.clicked.connect(self.save_outtake)

        btn_layout.addWidget(self.btn_save_intake)
        btn_layout.addWidget(self.btn_save_outtake)
        layout.addLayout(btn_layout)

        self.done_btn = QPushButton("Done / Close")
        self.done_btn.clicked.connect(self.accept)
        layout.addWidget(self.done_btn)

        self.setLayout(layout)

        # Connect signals
        for spin in [self.spin_x, self.spin_y, self.spin_z, self.spin_roll, self.spin_pitch, self.spin_yaw]:
            spin.valueChanged.connect(self.update_preview)

        self.scale_slider.valueChanged.connect(self.update_scale)

        self.update_preview()

    def create_spinbox(self, min_val, max_val, default):
        sb = QDoubleSpinBox()
        sb.setRange(min_val, max_val)
        sb.setSingleStep(0.05)
        sb.setValue(default)
        return sb

    # --- Update Game Piece Scale ---
    def update_scale(self, value):
        # Interpret percent -> scale factor; keep existing constant for the piece's units
        self.current_scale = value / 100.0
        self.scale_label.setText(f"Scale: {value}%")

        # Recreate the piece body (safe) using parent's robot pos as spawn
        try:
            # remove existing piece id if present
            if self.piece_id is not None:
                p.removeBody(self.piece_id)
        except Exception:
            pass

        scale = [self.current_scale] * 3

        # Rebuild from original stored STL path
        # NOTE: Game piece scaling is inherently less problematic as they are typically simple meshes.
        new_visual = p.createVisualShape(
            p.GEOM_MESH,
            fileName=self.mesh_path,
            meshScale=scale,
            rgbaColor=[1, 0.5, 0, 1]
        )

        new_collision = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=0.1 * self.current_scale   # optional: scale collision too
        )

        start_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        spawn_pos = [start_pos[0], start_pos[1], start_pos[2] + 0.5]

        self.piece_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=new_collision,
            baseVisualShapeIndex=new_visual,
            basePosition=spawn_pos
        )

        self.update_preview()


    def update_preview(self):
        self.current_pos = [self.spin_x.value(), self.spin_y.value(), self.spin_z.value()]
        self.current_rpy = [self.spin_roll.value(), self.spin_pitch.value(), self.spin_yaw.value()]

        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)

        offset_orn = p.getQuaternionFromEuler([
            self.current_rpy[0] * 3.14159/180,
            self.current_rpy[1] * 3.14159/180,
            self.current_rpy[2] * 3.14159/180
        ])

        final_pos, final_orn = p.multiplyTransforms(
            robot_pos, robot_orn,
            self.current_pos, offset_orn
        )

        try:
            p.resetBasePositionAndOrientation(self.piece_id, final_pos, final_orn)
        except Exception:
            pass

    def save_intake(self):
        self.intake_offset["pos"] = self.current_pos
        self.intake_offset["orn"] = self.current_rpy

    def save_outtake(self):
        self.outtake_offset["pos"] = self.current_pos
        self.outtake_offset["orn"] = self.current_rpy


# -------------------------
# View Cube (unchanged)
# -------------------------
class ViewCubeWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("View")
        self.setFixedSize(200, 200)

        layout = QVBoxLayout()

        self.up_btn = QPushButton("↑")
        self.down_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")

        for btn in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #f1c40f;
                    color: #1e1e1e;
                    font-size: 20px;
                    font-weight: bold;
                    border-radius: 8px;
                    padding: 5px;
                }
                QPushButton:hover {
                    background-color: #ffd700;
                }
            """)

        layout.addWidget(self.up_btn)
        row = QHBoxLayout()
        row.addWidget(self.left_btn)
        row.addWidget(self.right_btn)
        layout.addLayout(row)
        layout.addWidget(self.down_btn)

        self.setLayout(layout)

        self.up_btn.clicked.connect(lambda: self.adjust_camera("up"))
        self.down_btn.clicked.connect(lambda: self.adjust_camera("down"))
        self.left_btn.clicked.connect(lambda: self.adjust_camera("left"))
        self.right_btn.clicked.connect(lambda: self.adjust_camera("right"))

    def adjust_camera(self, direction):
        cam = p.getDebugVisualizerCamera()
        dist = cam[10]
        yaw = cam[8]
        pitch = cam[9]
        target = cam[11]

        if direction == "up":
            pitch = max(min(pitch - 10, 89), -89)
        elif direction == "down":
            pitch = max(min(pitch + 10, 89), -89)
        elif direction == "left":
            yaw -= 10
        elif direction == "right":
            yaw += 10

        p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, cameraPitch=pitch, cameraTargetPosition=target)


# -------------------------
# Main Simulator Class
# -------------------------
class FRCSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Project Skynest RPS - Team 4328 (Grant Andrews)")
        self.resize(700, 400)

        self.layout = QVBoxLayout()

        # Labels
        self.field_label = QLabel("Field: Not Loaded")
        self.robot_label = QLabel("Robot: Not Loaded")
        self.status_label = QLabel("")

        # Buttons
        self.load_field_btn = QPushButton("Load Field STL")
        self.load_robot_btn = QPushButton("Load Robot URDF")
        self.select_piece_btn = QPushButton("Select Game Piece")
        self.run_sim_btn = QPushButton("Run Simulation")

        self.load_field_btn.clicked.connect(self.load_field)

        self.load_robot_btn.clicked.connect(self.load_robot)
        self.import_process = QProcess(self)
        self.import_process.finished.connect(self.on_import_finished)

        self.select_piece_btn.clicked.connect(self.select_game_piece)
        self.run_sim_btn.clicked.connect(self.run_simulation)

        # Timeline table
        self.timeline = QTableWidget()
        self.timeline.setColumnCount(4)
        self.timeline.setHorizontalHeaderLabels(["Action", "Target", "Value", "Duration (s)"])
        self.timeline.setRowCount(5)
        self.timeline.cellChanged.connect(self.handle_cell_change)

        # Layout
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.load_field_btn)
        top_layout.addWidget(self.load_robot_btn)
        top_layout.addWidget(self.select_piece_btn)
        top_layout.addWidget(self.run_sim_btn)

        self.layout.addWidget(self.field_label)
        self.layout.addWidget(self.robot_label)
        self.layout.addLayout(top_layout)
        self.layout.addWidget(self.timeline)
        self.layout.addWidget(self.status_label)
        self.setLayout(self.layout)

        # PyBullet init
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        self.robot_id = None
        self.field_id = None
        self.field_mesh_path = None
        self.field_scale = 1.0
        self.field_euler = (0.0, 0.0, 0.0)
        self.field_base_position = [0,0,0.875]

        self.game_piece_id = None

        # Game Piece State
        self.game_piece_cid = None
        self.intake_config = {"pos": [0,0,0], "orn": [0,0,0]}
        self.outtake_config = {"pos": [0,0,0], "orn": [0,0,0]}

        self.joint_map = {}
        self.joint_default_positions = {}

        # Process for importer
        self.import_process = None

        self.view_cube_window = ViewCubeWindow()

    def show_view_cube(self):
        self.view_cube_window.show()

    def handle_cell_change(self, row, col):
        if row == self.timeline.rowCount() - 1:
            self.timeline.insertRow(self.timeline.rowCount())

    # -------------------------
    # Field loading + apply transform
    # -------------------------
    def load_field(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select Field STL", "", "STL Files (*.stl)")
        if not path:
            return

        self.field_mesh_path = path

        # Open adjust dialog FIRST
        dlg = FieldAdjustDialog(self, path)
        dlg.exec_()

        self.field_label.setText(f"Field: {os.path.basename(path)}")

    def apply_field_transform(self, mesh_path, scale, euler_deg, base_position=(0,0,0.875)):
        """
        Called live from FieldAdjustDialog.
        Applies scale + rotation + base position by rebuilding the field.
        """
        # OPTIMIZATION: If ONLY moving/rotating (scale is same), avoid destroying body.
        # This prevents resource exhaustion when dealing with a massive mesh.
        if (self.field_id is not None and 
            self.field_mesh_path == mesh_path and 
            abs(self.field_scale - scale) < 1e-5):
            
            # Convert degrees -> radians
            euler_rad = [x * 3.1415926 / 180.0 for x in euler_deg]
            quat = p.getQuaternionFromEuler(euler_rad)
            
            # Just move the existing body (cheap)
            p.resetBasePositionAndOrientation(self.field_id, base_position, quat)
            
            # Update internal state tracking
            self.field_euler = euler_deg
            self.field_base_position = base_position
            return

        # If scale changed or mesh path changed, we MUST recreate (The problematic step)
        self.field_mesh_path = mesh_path
        self.field_scale = scale
        self.field_euler = euler_deg
        self.field_base_position = base_position

        self._create_field_body(
            mesh_path,
            scale,
            euler_deg,
            base_position
        )

    def _create_field_body(self, mesh_path, scale, euler_deg, base_position):
        """
        Safely removes previous field, builds a new one with correct scale+orientation.
        Uses a simple box for collision to prevent memory exhaustion on scaling high-poly meshes.
        
        NOTE: If this function fails to create the body, the input STL file is too large
        and must be decimated (reduced in face count) externally.
        """

        # Delete old field - safely manage the ID
        if self.field_id is not None:
            try:
                p.removeBody(self.field_id)
            except Exception:
                pass
            self.field_id = None 

        # Convert degrees → radians → quaternion
        euler_rad = [x * 3.1415926 / 180.0 for x in euler_deg]
        quat = p.getQuaternionFromEuler(euler_rad)

        mesh_scale = [scale, scale, scale]

        try:
            # 1. VISUAL SHAPE (This is the costly step if the mesh is too large)
            visual = p.createVisualShape(
                shapeType=p.GEOM_MESH,
                fileName=mesh_path,
                meshScale=mesh_scale,
                rgbaColor=[.6, .6, .57, 1]
            )

            # 2. COLLISION SHAPE (Using a simple box proxy for collision/physics calculation)
            # FRC field dimensions are roughly 16.4m x 8.2m. We use a thin box.
            collision = p.createCollisionShape(
                shapeType=p.GEOM_BOX, 
                halfExtents=[16.4 / 2 * scale, 8.2 / 2 * scale, 0.05 * scale]
            )

            if visual == -1 or collision == -1:
                 raise ValueError("Failed to create visual or collision shape index.")
            
            self.field_id = p.createMultiBody(
                baseMass=0, # Static body
                baseVisualShapeIndex=visual,
                baseCollisionShapeIndex=collision,
                basePosition=base_position,
                baseOrientation=quat
            )
        except Exception as e:
            print(f"Error creating field body. This often means the STL file is too large (>200k faces) or improperly formatted: {e}")
            self.field_id = None

    # -------------------------
    # ROBOT loading logic (unchanged)
    # -------------------------
    def load_robot(self):
        self.status_label.setText("Launching Onshape importer...")
        QApplication.processEvents()

        self.import_process = QProcess(self)
        self.import_process.finished.connect(self.on_import_finished)

        # Launch the importer (user's exe) — keep prior behavior
        # If you bundle importer as a script inside the dist, call via sys.executable + resource path instead.
        self.import_process.start("onshape-to-robot-importer.exe")

    def on_import_finished(self):
        self.status_label.setText("Importer finished. Searching for URDF...")
        QApplication.processEvents()

        urdf_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select a URDF file",
            "",
            "URDF Files (*.urdf)"
        )

        if not urdf_path:
            self.status_label.setText("Error: No URDF found.")
            return

        self.load_urdf_from_path(urdf_path)

    def load_urdf_from_path(self, path):
        self.status_label.setText(f"Loading {path}...")
        QApplication.processEvents()
        
        if self.robot_id:
            p.removeBody(self.robot_id)
            
        try:
            urdf_dir = os.path.dirname(os.path.abspath(path))
            p.setAdditionalSearchPath(urdf_dir)
            self.robot_id = p.loadURDF(path, basePosition=[0, 0, 1])
            p.changeDynamics(self.robot_id, -1, mass=0)
            self.robot_label.setText(f"Robot: {os.path.basename(path)}")
        except p.error as e:
            QMessageBox.critical(self, "URDF Error", f"Could not load URDF: {e}")
            return

        self.build_joint_map()
        self.set_default_motor_control()
        self.status_label.setText("Robot loaded successfully.")

    def build_joint_map(self):
        self.joint_map = {}
        self.joint_default_positions = {}
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode("utf-8")
            cleaned = name.replace("dof_", "").replace("FRC_", "")
            self.joint_map[cleaned] = i
            self.joint_default_positions[i] = p.getJointState(self.robot_id, i)[0]
    
    def set_default_motor_control(self):
        for joint_idx, target_pos in self.joint_default_positions.items():
            p.setJointMotorControl2(
                self.robot_id, joint_idx, controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos, force=1000, positionGain=0.9
            )

    # -------------------------
    # GAME PIECE LOGIC (unchanged)
    # -------------------------
    def select_game_piece(self):
        if not self.robot_id:
            QMessageBox.warning(self, "Error", "Please load a robot first.")
            return

        path, _ = QFileDialog.getOpenFileName(self, "Select Game Piece STL", "", "STL Files (*.stl)")
        if not path:
            return

        self.game_piece_mesh_path = str(path)

        # Remove existing
        if self.game_piece_id:
            p.removeBody(self.game_piece_id)
            self.game_piece_id = None

        # Load new piece
        visual = p.createVisualShape(p.GEOM_MESH, fileName=self.game_piece_mesh_path, meshScale=[.00254/2]*3, rgbaColor=[1, 0.5, 0, 1])
        collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1) 
        
        start_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        spawn_pos = [start_pos[0], start_pos[1], start_pos[2] + 0.5]
        
        self.game_piece_id = p.createMultiBody(
            baseMass=0, 
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual, 
            basePosition=spawn_pos
        )
        
        dlg = GamePieceConfigDialog(self, self.robot_id, self.game_piece_id, self.game_piece_mesh_path)
        if dlg.exec_():
            self.intake_config = dlg.intake_offset
            self.outtake_config = dlg.outtake_offset
            self.select_piece_btn.setText("Game Piece Configured")
            self.set_piece_visible(False)
        else:
            p.removeBody(self.game_piece_id)
            self.game_piece_id = None

    def set_piece_visible(self, visible):
        if not self.game_piece_id: return
        if not visible:
            p.resetBasePositionAndOrientation(self.game_piece_id, [0, 0, -50], [0,0,0,1])

    def attach_game_piece(self, config):
        if not self.game_piece_id or not self.robot_id: return
        
        if self.game_piece_cid is not None:
            p.removeConstraint(self.game_piece_cid)
            self.game_piece_cid = None

        pos_offset = config["pos"]
        rpy = config["orn"]
        orn_offset = p.getQuaternionFromEuler([
            rpy[0] * 3.14159/180,
            rpy[1] * 3.14159/180,
            rpy[2] * 3.14159/180
        ])
        
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
        final_pos, final_orn = p.multiplyTransforms(robot_pos, robot_orn, pos_offset, orn_offset)
        p.resetBasePositionAndOrientation(self.game_piece_id, final_pos, final_orn)
        
        self.game_piece_cid = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.game_piece_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=pos_offset,
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=orn_offset
        )

    # -------------------------
    # Reset / Simulation (unchanged)
    # -------------------------
    def reset_joints(self):
        for joint_idx, default_pos in self.joint_default_positions.items():
            p.resetJointState(self.robot_id, joint_idx, default_pos)
            p.setJointMotorControl2(
                self.robot_id, joint_idx, controlMode=p.POSITION_CONTROL,
                targetPosition=default_pos, force=1000, positionGain=0.9
            )

    def run_simulation(self):
        if not self.robot_id:
            self.status_label.setText("Error: Load a robot first.")
            return

        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 1], [0, 0, 0, 1])
        self.reset_joints()
        
        # Reset Game Piece
        self.set_piece_visible(False)
        if self.game_piece_cid:
            p.removeConstraint(self.game_piece_cid)
            self.game_piece_cid = None

        self.status_label.setText("Running simulation...")
        QApplication.processEvents()

        total_time = 0.0
        for i in range(self.timeline.rowCount()):
            action = self.get_cell(i, 0).lower()
            target = self.get_cell(i, 1).lower()
            value = self.get_cell(i, 2)
            try:
                duration = float(self.get_cell(i, 3) or 1.0)
            except:
                duration = 1.0
            steps = int(duration * 60)

            # --- INTAKE/OUTTAKE LOGIC ---
            if action == "intake" and (target == "object" or target == "gamepiece"):
                if self.game_piece_id:
                    self.attach_game_piece(self.intake_config)
                time.sleep(duration)
            
            elif action == "outtake" and (target == "object" or target == "gamepiece"):
                if self.game_piece_id:
                    self.attach_game_piece(self.outtake_config)
                    time.sleep(duration)
                    if self.game_piece_cid:
                        p.removeConstraint(self.game_piece_cid)
                        self.game_piece_cid = None
                    self.set_piece_visible(False)
            
            elif action == "move":
                try:
                    dx, dy, dz = [float(x) for x in value.split()]
                except:
                    continue
                for _ in range(steps):
                    pos, orn = p.getBasePositionAndOrientation(self.robot_id)
                    new_pos = [pos[0] + dx/steps, pos[1] + dy/steps, pos[2] + dz/steps]
                    p.resetBasePositionAndOrientation(self.robot_id, new_pos, orn)
                    p.stepSimulation()
                    time.sleep(1 / 60.0)

            elif action == "rotate":
                try: amount = float(value)
                except: continue
                
                if target in self.joint_map:
                    joint_idx = self.joint_map[target]
                    start_angle = p.getJointState(self.robot_id, joint_idx)[0]
                    amount_rad = amount * 3.14159 / 180
                    for step in range(steps):
                        angle = start_angle + amount_rad * (step + 1) / steps
                        p.setJointMotorControl2(self.robot_id, joint_idx, p.POSITION_CONTROL, angle, force=1000)
                        p.stepSimulation()
                        time.sleep(1/60.0)
                else:
                    yaw_step = amount / steps
                    for _ in range(steps):
                        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
                        euler = p.getEulerFromQuaternion(orn)
                        new_yaw = euler[2] + yaw_step * 3.14159 / 180.0
                        new_orn = p.getQuaternionFromEuler([euler[0], euler[1], new_yaw])
                        p.resetBasePositionAndOrientation(self.robot_id, pos, new_orn)
                        p.stepSimulation()
                        time.sleep(1 / 60.0)

            elif action == "lift":
                try: distance = float(value)
                except: continue
                if target in self.joint_map:
                    joint_idx = self.joint_map[target]
                    start_pos = p.getJointState(self.robot_id, joint_idx)[0]
                    end_pos = start_pos + distance * 0.0254
                    for step in range(steps):
                        pos = start_pos + (end_pos - start_pos) * (step + 1) / steps
                        p.setJointMotorControl2(self.robot_id, joint_idx, p.POSITION_CONTROL, pos, force=1000)
                        p.stepSimulation()
                        time.sleep(1/60.0)

            total_time += duration

        self.status_label.setText(f"Cycle complete in {total_time:.2f} seconds")

    def get_cell(self, row, col):
        item = self.timeline.item(row, col)
        return item.text() if item else ""


# -------------------------
# App entrypoint
# -------------------------
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = FRCSimulator()

    dark_style = """
        QWidget { background-color: #1e1e1e; color: #f1f1f1; font-size: 14px; }
        QPushButton { background-color: #f1c40f; color: #1e1e1e; border-radius: 6px; padding: 5px 10px; font-weight: bold; }
        QPushButton:hover { background-color: #ffd700; }
        QPushButton:disabled { background-color: #444444; color: #888888; }
        QTableWidget { background-color: #2e2e2e; color: #ffffff; gridline-color: #f1c40f; }
        QHeaderView::section { background-color: #2e2e2e; color: #f1c40f; padding: 4px; border: none; }
        QLabel { color: #f1f1f1; }
        QLabel#FieldLabel, QLabel#RobotLabel { color: #f1c40f; font-weight: bold; }
        QGroupBox { border: 1px solid #f1c40f; margin-top: 20px; font-weight: bold; color: #f1c40f; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 3px; }
    """
    app.setStyleSheet(dark_style)
    win.field_label.setObjectName("FieldLabel")
    win.robot_label.setObjectName("RobotLabel")

    win.show()
    sys.exit(app.exec_())