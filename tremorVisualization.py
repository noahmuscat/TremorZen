import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QLineEdit, QComboBox, QFileDialog
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self, video_path, global_threshold, percentage_threshold, min_size_threshold, dilation_kernel, speed_option, width=0, height=0):
        super().__init__()
        self.video_path = video_path
        self.min_size_threshold = min_size_threshold
        self.global_threshold = global_threshold
        self.percentage_threshold = percentage_threshold
        self.dilation_kernel = dilation_kernel
        self.speed_option = speed_option
        self.width = width
        self.height = height
        self.speed_options_mapping = {"Slow": 100, "Normal": 30, "Fast": 10, "Very Fast": 1}
        self.is_running = True

        print(f"Initialized VideoThread with: video_path={video_path}, global_threshold={global_threshold}, "
            f"percentage_threshold={percentage_threshold}, min_size_threshold={min_size_threshold}, "
            f"dilation_kernel={dilation_kernel}, speed_option={speed_option}, width={width}, height={height}")

    def stop(self):
        self.is_running = False

    def calculate_metrics(self, frame, prev_frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prev_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)    
        	
        abs_diff = np.abs(frame - prev_frame)
        raw_diff = np.sum(abs_diff)
        rmse = (np.mean((frame - prev_frame) ** 2))**0.5
        
        
        #mean_diff = np.mean(abs_diff)
        #dynamic_threshold = mean_diff * (1 + global_threshold / 100.0)
        
        prev_frame_safe = prev_frame.astype(np.float32) + 1e-5
        frame = frame.astype(np.float32)
	    
	     # Calculate absolute difference and percentage change
        abs_diff = np.abs(frame - prev_frame)
        percentage_change = np.abs((frame - prev_frame_safe) / prev_frame_safe)
	    
	     # Scale percentage change to 0-100 (adjust scaling factor as needed)
        percentage_change_scaled = np.clip(percentage_change * 100, 0, 100).astype(np.uint8)
	    
	     # Apply thresholds to get binary masks
        _, abs_diff_mask = cv2.threshold(abs_diff, self.global_threshold, 255, cv2.THRESH_BINARY)
        _, percentage_change_mask = cv2.threshold(percentage_change_scaled, self.percentage_threshold, 100, cv2.THRESH_BINARY)
        abs_diff_mask = abs_diff_mask.astype(np.uint8)
        percentage_change_mask = percentage_change_mask.astype(np.uint8)

	     # Combine masks using logical AND to require both conditions to be met
        combined_mask = cv2.bitwise_and(abs_diff_mask, percentage_change_mask)
        
        
        kernel = np.ones((self.dilation_kernel, self.dilation_kernel), np.uint8)
        dilated_diff = cv2.dilate(combined_mask, kernel, iterations=1)
        dilated_diff = dilated_diff.astype(np.uint8)
        dilated_diff_filtered = self.filter_small_regions(dilated_diff, self.min_size_threshold)
        selected_pixel_diff = np.sum(dilated_diff_filtered)
        return raw_diff, rmse, selected_pixel_diff, dilated_diff_filtered

    def filter_small_regions(self, binary_image, min_size):
        #binary_image_single_channel = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
        filtered_image = np.zeros_like(binary_image)
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] >= min_size:
                filtered_image[labels == label] = 255
        return filtered_image

    def run(self):
        print("Starting video processing thread")
        cap = cv2.VideoCapture(self.video_path)
        prev_frame = None
        frame_delay = self.speed_options_mapping.get(self.speed_option, 30)

        frame_count = 0

        while self.is_running:
            ret, frame = cap.read()
            if not ret:
                print("End of video stream or error reading frame")
                break

            frame_count += 1
            
            # Resize frame if width and height are set
            if self.width > 0 and self.height > 0:
                frame = cv2.resize(frame, (self.width, self.height))
                print(f"Resized frame to: {self.width}x{self.height}")

            if prev_frame is not None:
                raw_diff, rmse, selected_pixel_diff, dilated_diff_filtered = self.calculate_metrics(frame, prev_frame)
                frame_highlighted = np.copy(frame)
                frame_highlighted[dilated_diff_filtered > 0] = [0, 255, 0]
                self.change_pixmap_signal.emit(frame_highlighted)
                print(f"Processed frame {frame_count}: raw_diff={raw_diff}, rmse={rmse}, selected_pixel_diff={selected_pixel_diff}")

            prev_frame = frame
            cv2.waitKey(frame_delay)

        cap.release()
        print("Stopped video processing thread")

class VideoAnalyzerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Video Analyzer")
        self.video_path = ""
        self.thread = None
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.layout.addWidget(self.image_label)

        # Creating a function to add entries with labels for cleaner code
        def add_entry_with_label(layout, label_text, default_value=""):
            entry_layout = QVBoxLayout()
            entry = QLineEdit(default_value)
            label = QLabel(label_text)
            entry_layout.addWidget(entry)
            entry_layout.addWidget(label)
            layout.addLayout(entry_layout)
            return entry

        controls_layout = QHBoxLayout()

        # Adding video path entry
        self.video_path_entry = QLineEdit()
        controls_layout.addWidget(self.video_path_entry)
        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.browse_video)
        controls_layout.addWidget(self.browse_button)

        # Adding entries with labels
        self.global_threshold_entry = add_entry_with_label(controls_layout, "Global Threshold", "15")
        self.percentage_threshold_entry = add_entry_with_label(controls_layout, "Percentage Threshold", "25")
        self.min_size_entry = add_entry_with_label(controls_layout, "Min Size", "120")
        self.kernel_entry = add_entry_with_label(controls_layout, "Kernel Size", "4")
        
        # Adding width and height entries with labels
        self.width_entry = add_entry_with_label(controls_layout, "Video Width", "0")
        self.height_entry = add_entry_with_label(controls_layout, "Video Height", "0")

        # Confirmation button for initialization
        self.confirm_button = QPushButton("Confirm Dimensions")
        self.confirm_button.clicked.connect(self.confirm_dimensions)
        controls_layout.addWidget(self.confirm_button)

        # Initializing confirmed width and height to be used in the thread
        self.confirmed_width = 0
        self.confirmed_height = 0

        # Speed selection
        self.speed_var = QComboBox()
        self.speed_var.addItems(["Slow", "Normal", "Fast", "Very Fast"])
        controls_layout.addWidget(self.speed_var)

        # Run and Stop buttons
        self.run_button = QPushButton("Run Analysis")
        self.run_button.clicked.connect(self.run_analysis)
        controls_layout.addWidget(self.run_button)

        self.stop_button = QPushButton("Stop Analysis")
        self.stop_button.clicked.connect(self.stop_analysis)
        controls_layout.addWidget(self.stop_button)

        self.layout.addLayout(controls_layout)
        self.setLayout(self.layout)

        print("Initialized UI with controls")

    def confirm_dimensions(self):
        try:
            self.confirmed_width = int(self.width_entry.text())
            self.confirmed_height = int(self.height_entry.text())
            print(f"Confirmed dimensions: width={self.confirmed_width}, height={self.confirmed_height}")
        except ValueError as ve:
            print("Please enter valid integer values for width and height.", ve)

    def browse_video(self):
        video_path, _ = QFileDialog.getOpenFileName(self, "Select Video", "", "*.mp4")
        if video_path:
            self.video_path_entry.setText(video_path)
            self.video_path = video_path

    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        qt_img = QPixmap.fromImage(convert_to_qt_format).scaled(self.image_label.width(), self.image_label.height(), Qt.KeepAspectRatio)
        return qt_img

    def run_analysis(self):
        if self.thread is not None and self.thread.isRunning():
            self.thread.stop()
        min_size_threshold = int(self.min_size_entry.text())
        global_threshold = int(self.global_threshold_entry.text())
        percentage_threshold = int(self.percentage_threshold_entry.text())
        dilation_kernel = int(self.kernel_entry.text())
        speed_option = self.speed_var.currentText()
        width = self.confirmed_width
        height = self.confirmed_height
        
        print(f"Running analysis with: min_size_threshold={min_size_threshold}, global_threshold={global_threshold}, "
            f"percentage_threshold={percentage_threshold}, dilation_kernel={dilation_kernel}, speed_option={speed_option}, "
            f"width={width}, height={height}")

        self.thread = VideoThread(self.video_path, global_threshold, percentage_threshold, min_size_threshold, dilation_kernel, speed_option, width, height)
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def stop_analysis(self):
        if self.thread is not None:
            self.thread.stop()
            self.thread.wait()  # Ensure the thread has finished
            self.image_label.clear()  # Clear the image display

def main():
    app = QApplication(sys.argv)
    main_window = VideoAnalyzerApp()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()