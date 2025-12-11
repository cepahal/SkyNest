import sys

import os

import json

import subprocess

import re

from PyQt5.QtWidgets import (

    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,

    QLabel, QLineEdit, QPushButton, QTextEdit, QProgressBar, QMessageBox,

    QFrame, QGraphicsDropShadowEffect

)

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPropertyAnimation, QEasingCurve, QPoint

from PyQt5.QtGui import QColor, QFont, QIcon



def get_onshape_to_robot_executable():

    # When running from PyInstaller, the script is inside _MEIPASS

    exe_filename = "onshape-to-robot.exe" if os.name == "nt" else "onshape-to-robot"

    if hasattr(sys, "_MEIPASS"):

        bundled_path = os.path.join(sys._MEIPASS, "_internal", exe_filename)

        if os.path.exists(bundled_path):

            return bundled_path

    # Fallback: try same directory as executable

    local_path = os.path.join(os.path.dirname(sys.executable), "_internal", exe_filename)

    if os.path.exists(local_path):

        return local_path

    # Last fallback: PATH

    return exe_filename



# --- STYLING ---

STYLESHEET = """

QMainWindow {

    background-color: #121212;

}

QLabel {

    color: #E0E0E0;

    font-size: 14px;

    font-family: 'Segoe UI', sans-serif;

}

QLineEdit {

    background-color: #1E1E1E;

    border: 2px solid #333333;

    border-radius: 8px;

    color: #FFFFFF;

    padding: 10px;

    font-size: 14px;

    selection-background-color: #BB86FC;

}

QLineEdit:focus {

    border: 2px solid #BB86FC;

}

QPushButton {

    background-color: #3700B3;

    color: white;

    border-radius: 8px;

    padding: 12px;

    font-weight: bold;

    font-size: 15px;

}

QPushButton:hover {

    background-color: #6200EE;

}

QPushButton:pressed {

    background-color: #3700B3;

}

QPushButton:disabled {

    background-color: #333333;

    color: #888888;

}

QTextEdit {

    background-color: #000000;

    color: #00FF41; /* Hacker Green */

    font-family: 'Consolas', 'Courier New', monospace;

    border: 1px solid #333333;

    border-radius: 5px;

    padding: 5px;

}

QProgressBar {

    border: 2px solid #333333;

    border-radius: 5px;

    text-align: center;

    background-color: #1E1E1E;

    color: white;

}

QProgressBar::chunk {

    background-color: #BB86FC;

    border-radius: 3px;

}

"""



class WorkerThread(QThread):

    log_signal = pyqtSignal(str)

    finished_signal = pyqtSignal(bool)

   

    def __init__(self, directory, access_key, secret_key, doc_url):

        super().__init__()

        self.directory = directory

        self.access_key = access_key

        self.secret_key = secret_key

        self.doc_url = doc_url





    def extract_doc_id(self, url):

        # Extracts ID from website

        try:

            pattern = r"/documents/([a-zA-Z0-9]+)"

            match = re.search(pattern, url)

            if match:

                return match.group(1)

            return None

        except:

            return None



    def run(self):

        try:

            # 1. Validation

            self.log_signal.emit(">>> Validating inputs...")

            doc_id = self.extract_doc_id(self.doc_url)

            if not doc_id:

                self.log_signal.emit("ERROR: Could not find Document ID in URL.")

                self.finished_signal.emit(False)

                return



            if not self.directory:

                self.log_signal.emit("ERROR: Directory name cannot be empty.")

                self.finished_signal.emit(False)

                return



            # Create Directory

            target_dir = os.path.abspath(self.directory)

            if not os.path.exists(target_dir):

                self.log_signal.emit(f">>> Creating directory: {target_dir}")

                os.makedirs(target_dir)

            else:

                self.log_signal.emit(f">>> Directory exists: {target_dir}")



            # Create Config JSON

            self.log_signal.emit(">>> Generating config.json...")

            config_data = {

                "documentId": doc_id,

                "outputFormat": "urdf",

                "drawFrames": False,

                "useFixedLinks": False

            }

           

            config_path = os.path.join(target_dir, "config.json")

            with open(config_path, 'w') as f:

                json.dump(config_data, f, indent=4)

            self.log_signal.emit(f"   Saved config to {config_path}")



            # Prepare Environment

            env = os.environ.copy()

            env["ONSHAPE_API"] = "https://cad.onshape.com"

            env["ONSHAPE_ACCESS_KEY"] = self.access_key

            env["ONSHAPE_SECRET_KEY"] = self.secret_key



            # Run onshape-to-robot

            self.log_signal.emit(">>> Starting onshape-to-robot process...")

            self.log_signal.emit("   (This may take a few minutes)")

           

            # ATTEMPT TO FIND THE EXECUTABLE SCRIPT

            script_name = "onshape-to-robot"

            if os.name == 'nt':

                script_name += ".exe"

           

            # Look in Scripts (Windows) or bin (Unix) relative to the python interpreter

            possible_paths = [

                os.path.join(os.path.dirname(sys.executable), 'Scripts', script_name),

                os.path.join(os.path.dirname(sys.executable), 'bin', script_name),

                script_name # Fallback to PATH

            ]

           

            cmd_executable = get_onshape_to_robot_executable()

           

            self.log_signal.emit(f"   Using command: {cmd_executable}")

           

            cmd = [cmd_executable, target_dir]

           

            process = subprocess.Popen(

                cmd,

                stdout=subprocess.PIPE,

                stderr=subprocess.PIPE,

                text=True,

                env=env,

                bufsize=1

            )



            # Stream output

            for line in process.stdout:

                self.log_signal.emit(line.strip())

           

            # Capture errors

            stderr = process.communicate()[1]

            if stderr:

                self.log_signal.emit(f"STDERR: {stderr}")



            if process.returncode == 0:

                self.log_signal.emit(">>> SUCCESS: Robot converted successfully!")

                self.finished_signal.emit(True)

            else:

                self.log_signal.emit(">>> FAILURE: Process exited with errors.")

                self.finished_signal.emit(False)



        except Exception as e:

            self.log_signal.emit(f"CRITICAL ERROR: {str(e)}")

            self.finished_signal.emit(False)



class EasyOnshapeGUI(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("Press X when finished or if selecting existing URDF")

        self.resize(600, 750)

        self.setStyleSheet(STYLESHEET)

       

        # Main Layout

        central_widget = QWidget()

        self.setCentralWidget(central_widget)

        self.layout = QVBoxLayout(central_widget)

        self.layout.setSpacing(15)

        self.layout.setContentsMargins(30, 30, 30, 30)



        # Header

        title = QLabel("Onshape URDF Converter")

        title.setFont(QFont("Segoe UI", 20, QFont.Bold))

        title.setAlignment(Qt.AlignCenter)

        title.setStyleSheet("color: #BB86FC; margin-bottom: 10px;")

        self.layout.addWidget(title)



        # Inputs

        self.dir_input = self.create_input("Project Name (Created):", "e.g., my-robot")

        self.url_input = self.create_input("Onshape Document URL:", "Paste full URL (https://cad.onshape.com/documents/...)")

        self.access_input = self.create_input("Access Key:", "Your API Access Key")

        self.secret_input = self.create_input("Secret Key:", "Your API Secret Key", is_password=True)



        # Convert Button

        self.convert_btn = QPushButton("START CONVERSION")

        self.convert_btn.setCursor(Qt.PointingHandCursor)

        self.convert_btn.clicked.connect(self.start_process)

       

        # Shadow Effect for button

        shadow = QGraphicsDropShadowEffect()

        shadow.setBlurRadius(15)

        shadow.setColor(QColor(187, 134, 252, 100))

        shadow.setOffset(0, 0)

        self.convert_btn.setGraphicsEffect(shadow)

       

        self.layout.addWidget(self.convert_btn)



        # Progress Bar (Indeterminate)

        self.progress = QProgressBar()

        self.progress.setRange(0, 100)

        self.progress.setValue(0)

        self.progress.setTextVisible(False)

        self.progress.setFixedHeight(5)

        self.layout.addWidget(self.progress)



        # Terminal Output

        term_label = QLabel("Conversion Log:")

        term_label.setStyleSheet("font-weight: bold; margin-top: 10px;")

        self.layout.addWidget(term_label)

       

        self.terminal = QTextEdit()

        self.terminal.setReadOnly(True)

        self.terminal.setPlaceholderText("Waiting for input...")

        self.layout.addWidget(self.terminal)



        self.selected_urdf_path = None



       



    def create_input(self, label_text, placeholder, is_password=False):

        container = QWidget()

        vbox = QVBoxLayout(container)

        vbox.setContentsMargins(0, 0, 0, 0)

        vbox.setSpacing(5)

       

        label = QLabel(label_text)

        inp = QLineEdit()

        inp.setPlaceholderText(placeholder)

        if is_password:

            inp.setEchoMode(QLineEdit.Password)

       

        vbox.addWidget(label)

        vbox.addWidget(inp)

        self.layout.addWidget(container)

        return inp



    def start_process(self):

        # 1. Gather Data

        directory = self.dir_input.text().strip()

        url = self.url_input.text().strip()

        access = self.access_input.text().strip()

        secret = self.secret_input.text().strip()



        if not all([directory, url, access, secret]):

            QMessageBox.warning(self, "Missing Input", "Please fill in all fields.")

            return



        # 2. UI Updates

        self.terminal.clear()

        self.convert_btn.setEnabled(False)

        self.convert_btn.setText("CONVERTING...")

        self.progress.setRange(0, 0) # Infinite loading animation



        # 3. Start Thread

        self.worker = WorkerThread(directory, access, secret, url)

        self.worker.log_signal.connect(self.append_log)

        self.worker.finished_signal.connect(self.process_finished)

        self.worker.start()



    def append_log(self, text):

        self.terminal.append(text)

        # Auto scroll to bottom

        sb = self.terminal.verticalScrollBar()

        sb.setValue(sb.maximum())



    def process_finished(self, success):

        self.convert_btn.setEnabled(True)

        self.convert_btn.setText("START CONVERSION")

        self.progress.setRange(0, 100)

       

        if success:

            self.progress.setValue(100)

            self.progress.setStyleSheet("QProgressBar::chunk { background-color: #00FF41; }") # Green

            QMessageBox.information(self, "Success", "Robot exported successfully!")

        else:

            self.progress.setValue(100)

            self.progress.setStyleSheet("QProgressBar::chunk { background-color: #CF6679; }") # Red

            QMessageBox.critical(self, "Failed", "The process encountered errors. Check the log.")



if __name__ == '__main__':

    app = QApplication(sys.argv)

    window = EasyOnshapeGUI()

    window.show()

    sys.exit(app.exec_())