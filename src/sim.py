import sys
import os
import pybullet as p
import pybullet_data
import time
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QFileDialog, QTableWidget, QTableWidgetItem, QLabel, QHBoxLayout,
    QMessageBox, QDialog, QSlider, QFormLayout, QDoubleSpinBox, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer, QProcess

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

        # NEW: Track scale (1.0 = 100%)
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
        self.current_scale = value * .0000127
        self.scale_label.setText(f"Scale: {value}%")

        # You MUST have selected a game piece first
        if not hasattr(self, "mesh_path"):
            return

        # Remove old body
        p.removeBody(self.piece_id)

        scale = [self.current_scale] * 3

        # Rebuild from original stored STL path
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

        # Respawn the game piece near the robot
        start_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        spawn_pos = [start_pos[0], start_pos[1], start_pos[2] + 0.5]

        self.game_piece_id = p.createMultiBody(
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

        p.resetBasePositionAndOrientation(self.piece_id, final_pos, final_orn)

    def save_intake(self):
        self.intake_offset["pos"] = self.current_pos
        self.intake_offset["orn"] = self.current_rpy

    def save_outtake(self):
        self.outtake_offset["pos"] = self.current_pos
        self.outtake_offset["orn"] = self.current_rpy


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

class FRCSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Project Skynest RPS - Team 4328 (Grant Andrews)")
        self.resize(600, 400)

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

    def load_field(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Field STL", "", "STL Files (*.stl)")
        if path:
            if self.field_id: p.removeBody(self.field_id)
            visual = p.createVisualShape(p.GEOM_MESH, fileName=path, meshScale=[1]*3, rgbaColor=[0.65, .65, 0.65, 1])
            collision = p.createCollisionShape(p.GEOM_MESH, fileName=path, meshScale=[0,0,0])
            self.field_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=[0, 0, .875])
            self.field_label.setText(f"Field: {os.path.basename(path)}")
            self.show_view_cube()

    # --- LOADING ROBOT ---
    
    def load_robot(self):
        self.status_label.setText("Launching Onshape importer...")
        QApplication.processEvents()

        self.import_process = QProcess(self)
        self.import_process.finished.connect(self.on_import_finished)

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

        if urdf_path is None:
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
            # FRC_* or dof_FRC_* cleanup
            cleaned = name.replace("dof_", "").replace("FRC_", "")
            self.joint_map[cleaned] = i
            self.joint_default_positions[i] = p.getJointState(self.robot_id, i)[0]
    
    def set_default_motor_control(self):
        # Apply high-force position control to act as a strong brake/servo, 
        # ensuring joints hold their position relative to the base, even when the base moves.
        for joint_idx, target_pos in self.joint_default_positions.items():
            p.setJointMotorControl2(
                self.robot_id, joint_idx, controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos, force=1000, positionGain=0.9
            )

    # --- GAME PIECE LOGIC ---
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