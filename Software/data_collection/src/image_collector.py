"""
Modern image collection tool for YOLO dataset creation
"""

import os
import sys
from datetime import datetime

import cv2
from PySide6.QtCore import QMutex, QMutexLocker, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QFont, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class CameraWorker(QThread):
    """Thread worker for camera operations"""

    frame_ready = Signal(object)  # Signal to send frame to main thread
    capture_ready = Signal(str)  # Signal to notify image saved

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cap = None
        self.running = False
        self.paused = False
        self.capture_active = False
        self.capture_mutex = QMutex()
        self.image_prefix = ""
        self.image_count = 0
        self.max_images = (
            120  # Default value, but can be changed via set_capture_settings
        )
        self.output_dir = ""

    def start_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")
        self.running = True
        self.paused = False
        self.start()

    def stop_camera(self):
        self.running = False
        if self.cap:
            self.cap.release()
        self.wait()

    def pause_camera(self):
        self.paused = True

    def resume_camera(self):
        self.paused = False

    def set_capture_settings(self, prefix, output_dir, max_images=120, start_count=0):
        self.image_prefix = prefix
        self.output_dir = output_dir
        self.max_images = max_images
        self.image_count = start_count

    def trigger_capture(self):
        with QMutexLocker(self.capture_mutex):
            if not self.paused:
                self.capture_active = True

    def run(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    # Emit the frame for display
                    if not self.paused:
                        self.frame_ready.emit(frame)

                    # Check if capture is active and not at max and not paused
                    if (
                        self.capture_active
                        and self.image_count < self.max_images
                        and not self.paused
                    ):
                        with QMutexLocker(self.capture_mutex):
                            self.capture_active = False

                        # Save the image
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[
                            :-3
                        ]  # ms precision
                        image_name = f"{self.image_prefix}_{self.image_count + 1:03d}_{timestamp}.jpg"
                        image_path = os.path.join(self.output_dir, image_name)

                        # Save image with high quality
                        success = cv2.imwrite(
                            image_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95]
                        )
                        if success:
                            self.image_count += 1
                            self.capture_ready.emit(image_path)

                        # Auto-stop if reached max images
                        if self.image_count >= self.max_images:
                            self.capture_ready.emit("MAX_IMAGES_REACHED")
            # Small delay to control frame rate
            self.msleep(30)  # ~33 FPS


class ImageCollectorUI(QMainWindow):
    """Main UI for the image collection tool"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("YOLO Image Collector")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize camera worker
        self.camera_worker = CameraWorker()
        self.camera_worker.frame_ready.connect(self.update_frame)
        self.camera_worker.capture_ready.connect(self.on_capture_complete)

        # UI elements
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet(
            "background-color: black; border: 1px solid gray;"
        )

        self.last_image_label = QLabel("No image captured yet")
        self.last_image_label.setAlignment(Qt.AlignCenter)
        self.last_image_label.setMinimumSize(320, 240)
        self.last_image_label.setStyleSheet(
            "background-color: lightgray; border: 1px solid gray;"
        )

        # Status label
        self.status_label = QLabel("Ready to start")
        font = QFont()
        font.setBold(True)
        self.status_label.setFont(font)
        self.status_label.setAlignment(Qt.AlignCenter)

        # Controls
        self.color_input = QLineEdit()
        self.color_input.setPlaceholderText("Enter color (e.g., red, blue)")
        self.color_input.setText("blue")  # Default value

        # Maximum images setting
        self.max_images_spinbox = QSpinBox()
        self.max_images_spinbox.setRange(1, 9999)  # Range from 1 to 9999 images
        self.max_images_spinbox.setValue(120)  # Default value
        self.max_images_spinbox.setSuffix(" images")

        self.output_dir_btn = QPushButton("Select Output Folder")
        self.output_dir_btn.clicked.connect(self.select_output_folder)

        self.output_dir_label = QLabel("No folder selected")
        self.output_dir_label.setWordWrap(True)

        self.start_btn = QPushButton("Start Camera")
        self.start_btn.clicked.connect(self.start_camera)
        self.start_btn.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;"
        )

        self.stop_btn = QPushButton("Stop Camera")
        self.stop_btn.clicked.connect(self.stop_camera)
        self.stop_btn.setStyleSheet(
            "background-color: #f44336; color: white; font-weight: bold; padding: 10px;"
        )
        self.stop_btn.setEnabled(False)

        self.capture_btn = QPushButton("Capture Image (Space)")
        self.capture_btn.clicked.connect(self.capture_image)
        self.capture_btn.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        )
        self.capture_btn.setEnabled(False)

        # Image counter
        self.counter_label = QLabel("Captured: 0 / 120")

        # Frequency control
        self.freq_spinbox = QSpinBox()
        self.freq_spinbox.setRange(1, 10)  # 1-10 seconds
        self.freq_spinbox.setValue(2)  # Default 2 seconds
        self.freq_spinbox.setSuffix(" sec")

        # Auto-capture timer
        self.auto_capture_timer = QTimer()
        self.auto_capture_timer.timeout.connect(self.capture_image)

        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout(central_widget)

        # Left side: video feed
        left_layout = QVBoxLayout()
        video_group = QGroupBox("Camera Feed")
        video_layout = QVBoxLayout()
        video_layout.addWidget(self.video_label)
        video_group.setLayout(video_layout)

        last_img_group = QGroupBox("Last Captured Image")
        last_img_layout = QVBoxLayout()
        last_img_layout.addWidget(self.last_image_label)
        last_img_group.setLayout(last_img_layout)

        left_layout.addWidget(video_group)
        left_layout.addWidget(last_img_group)
        left_layout.addWidget(self.status_label)

        # Right side: controls
        right_layout = QVBoxLayout()

        # Color and folder selection
        config_group = QGroupBox("Configuration")
        config_layout = QFormLayout()
        config_layout.addRow("Color Prefix:", self.color_input)
        config_layout.addRow("Max Images:", self.max_images_spinbox)
        config_layout.addRow(self.output_dir_btn, self.output_dir_label)
        config_group.setLayout(config_layout)

        # Manual capture
        manual_group = QGroupBox("Manual Capture")
        manual_layout = QVBoxLayout()
        manual_layout.addWidget(self.capture_btn)
        manual_group.setLayout(manual_layout)

        # Auto capture
        auto_group = QGroupBox("Auto Capture")
        auto_layout = QFormLayout()
        auto_layout.addRow("Capture Frequency:", self.freq_spinbox)
        auto_group.setLayout(auto_layout)

        # Controls
        controls_group = QGroupBox("Controls")
        controls_layout = QVBoxLayout()
        controls_layout.addWidget(self.start_btn)
        controls_layout.addWidget(self.stop_btn)

        # Add pause/resume button
        self.pause_btn = QPushButton("Pause Camera")
        self.pause_btn.clicked.connect(self.pause_camera)
        self.pause_btn.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold; padding: 10px;"
        )
        self.pause_btn.setEnabled(False)
        controls_layout.addWidget(self.pause_btn)

        controls_group.setLayout(controls_layout)

        # Counter
        counter_group = QGroupBox("Progress")
        counter_layout = QVBoxLayout()
        counter_layout.addWidget(self.counter_label)
        counter_group.setLayout(counter_layout)

        right_layout.addWidget(config_group)
        right_layout.addWidget(manual_group)
        right_layout.addWidget(auto_group)
        right_layout.addWidget(controls_group)
        right_layout.addWidget(counter_group)
        right_layout.addStretch()

        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 1)

        # Keyboard shortcuts
        self.setTabOrder(self.color_input, self.output_dir_btn)
        self.setTabOrder(self.output_dir_btn, self.capture_btn)
        self.setTabOrder(self.capture_btn, self.start_btn)
        self.setTabOrder(self.start_btn, self.stop_btn)

        # Handle spacebar for capture
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space and self.capture_btn.isEnabled():
            self.capture_image()
        else:
            super().keyPressEvent(event)

    def select_output_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Output Folder")
        if folder:
            self.output_dir_label.setText(folder)
            self.output_dir = folder

    def start_camera(self):
        if not hasattr(self, "output_dir") or not self.output_dir:
            self.status_label.setText("Please select an output folder first!")
            return

        color_prefix = self.color_input.text().strip()
        if not color_prefix:
            self.status_label.setText("Please enter a color prefix!")
            return

        try:
            # Calculate starting count based on existing images with the same prefix
            existing_count = self._get_existing_image_count(color_prefix)

            # Get max images from the spinbox
            max_images = self.max_images_spinbox.value()

            # Set capture settings with starting count and max images
            self.camera_worker.set_capture_settings(
                color_prefix,
                self.output_dir,
                max_images=max_images,
                start_count=existing_count,
            )

            # Start camera
            self.camera_worker.start_camera()

            # Update UI
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.pause_btn.setEnabled(True)  # Enable pause button
            self.capture_btn.setEnabled(True)
            self.counter_label.setText(f"Captured: {existing_count} / {max_images}")
            self.status_label.setText(
                f"Camera running - {existing_count} images already exist, press SPACE to capture or use Auto Capture"
            )

            # Start auto capture timer if frequency > 0
            freq = self.freq_spinbox.value()
            if freq > 0:
                self.auto_capture_timer.start(freq * 1000)  # Convert to milliseconds

        except Exception as e:
            self.status_label.setText(f"Error starting camera: {str(e)}")

    def _get_existing_image_count(self, prefix):
        """Count existing images with the given prefix in the output directory"""
        if not os.path.exists(self.output_dir):
            return 0

        count = 0
        for filename in os.listdir(self.output_dir):
            if filename.startswith(f"{prefix}_") and filename.endswith(".jpg"):
                try:
                    # Extract the number from filename like "prefix_001_..."
                    parts = filename.split("_")
                    if len(parts) >= 2:
                        num_part = parts[1]
                        # Check if it's a number
                        int(num_part)
                        count += 1
                except ValueError:
                    # If conversion to int fails, skip this file
                    continue
        return count

    def stop_camera(self):
        self.camera_worker.stop_camera()
        self.auto_capture_timer.stop()

        # Update UI
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.pause_btn.setEnabled(False)
        self.capture_btn.setEnabled(False)
        self.status_label.setText("Camera stopped")

        # Clear video display
        self.video_label.clear()
        self.video_label.setText("Camera feed stopped")

    def pause_camera(self):
        if self.camera_worker.running and not self.camera_worker.paused:
            self.camera_worker.pause_camera()
            self.pause_btn.setText("Resume Camera")
            self.pause_btn.setStyleSheet(
                "background-color: #8BC34A; color: white; font-weight: bold; padding: 10px;"
            )
            self.status_label.setText("Camera paused - press Resume to continue")
            self.capture_btn.setEnabled(False)
        elif self.camera_worker.running and self.camera_worker.paused:
            self.camera_worker.resume_camera()
            self.pause_btn.setText("Pause Camera")
            self.pause_btn.setStyleSheet(
                "background-color: #FF9800; color: white; font-weight: bold; padding: 10px;"
            )
            self.status_label.setText(
                f"Camera running - {self.camera_worker.image_count} images captured so far"
            )
            self.capture_btn.setEnabled(True)

    def capture_image(self):
        if self.camera_worker.running:
            self.camera_worker.trigger_capture()

    def update_frame(self, frame):
        # Convert frame to QImage
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(q_img)

        # Scale pixmap to fit label while maintaining aspect ratio
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )

        self.video_label.setPixmap(scaled_pixmap)

    def on_capture_complete(self, result):
        if result == "MAX_IMAGES_REACHED":
            max_images = self.camera_worker.max_images
            self.status_label.setText(
                f"Max images ({max_images}) reached. Stopping capture."
            )
            self.stop_camera()
        else:
            # Update counter
            count = self.camera_worker.image_count
            max_images = self.camera_worker.max_images
            self.counter_label.setText(f"Captured: {count} / {max_images}")

            # Load and display the last captured image
            if os.path.exists(result):
                pixmap = QPixmap(result)
                if not pixmap.isNull():
                    scaled_pixmap = pixmap.scaled(
                        self.last_image_label.size(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                    self.last_image_label.setPixmap(scaled_pixmap)

            self.status_label.setText(
                f"Captured image: {os.path.basename(result)} | Total: {count}"
            )

    def closeEvent(self, event):
        # Ensure camera is stopped before closing
        if self.camera_worker.running:
            self.camera_worker.stop_camera()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = ImageCollectorUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
