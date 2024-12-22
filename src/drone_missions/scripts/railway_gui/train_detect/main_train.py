import sys
import rospy
import cv2
import numpy as np
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QGridLayout
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, Qt
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from line_detect import LineDetector
from drone_control import DroneControl
from fusion_downLidarANDcamera import process_fusion_downward  # Füzyon işlemi için import

class FusionGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Çizgi Takibi Arayüzü")
        self.setGeometry(100, 100, 800, 600)

        # QLabel'ler
        self.downward_camera_label = QLabel("Çizgi tespiti bekleniyor...")
        self.downward_camera_label.setAlignment(Qt.AlignCenter)

        self.status_label = QLabel("Durum: Bekleniyor...")
        self.status_label.setAlignment(Qt.AlignCenter)

        # Drone kontrolü için sınıf
        self.drone_control = DroneControl()

        # ROS bağlantıları
        self.bridge = CvBridge()
        self.last_downward_camera_msg = None
        self.last_lidar_msg_downward = None

        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.downward_camera_callback)
        rospy.Subscriber("/scan/downward", LaserScan, self.lidar_callback)  # Lidar mesajı dinleyici

        # LineDetector parametreleri
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        self.line_detector = LineDetector(lower_red1, upper_red1, lower_red2, upper_red2)

        # Görüntü güncelleme için QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_views)
        self.timer.start(100)  # 100 ms

        # Layout ayarı
        layout = QVBoxLayout()
        layout.addWidget(self.downward_camera_label)

        # Kontrol butonları için QGridLayout
        grid_layout = QGridLayout()

        # Butonlar için konumlandırma (row, column)
        grid_layout.addWidget(QPushButton("Yukarı", clicked=lambda: self.drone_control.up_fun(self.status_label)), 2, 2)
        grid_layout.addWidget(QPushButton("Aşağı", clicked=lambda: self.drone_control.down_fun(self.status_label)), 2, 0)
        grid_layout.addWidget(QPushButton("İleri", clicked=lambda: self.drone_control.forward_fun(self.status_label)), 0, 1)
        grid_layout.addWidget(QPushButton("Geri", clicked=lambda: self.drone_control.backward_fun(self.status_label)), 2, 1)
        grid_layout.addWidget(QPushButton("Sağa", clicked=lambda: self.drone_control.right_fun(self.status_label)), 0, 2)
        grid_layout.addWidget(QPushButton("Sola", clicked=lambda: self.drone_control.left_fun(self.status_label)), 0, 0)
        grid_layout.addWidget(QPushButton("Saat Yönü", clicked=lambda: self.drone_control.cw_fun(self.status_label)), 1, 2)
        grid_layout.addWidget(QPushButton("Saat Tersi", clicked=lambda: self.drone_control.ccw_fun(self.status_label)), 1, 0)
        grid_layout.addWidget(QPushButton("Sabit", clicked=lambda: self.drone_control.hover_pub(self.status_label)), 1, 1)

        # Ana layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addWidget(self.status_label)
        main_layout.addLayout(grid_layout)
        self.setLayout(main_layout)

    def downward_camera_callback(self, msg):
        self.last_downward_camera_msg = msg

    def lidar_callback(self, msg):
        self.last_lidar_msg_downward = msg

    def update_views(self):
        if self.last_downward_camera_msg and self.last_lidar_msg_downward:
            self.update_downward_view()

    def update_downward_view(self):
        # Kameradan alınan görüntü
        cv_image = self.bridge.imgmsg_to_cv2(self.last_downward_camera_msg, "bgr8")

        # Lidar verileri ile füzyon
        fused_frame = process_fusion_downward(self.last_lidar_msg_downward, self.last_downward_camera_msg)

        # Line detect işlemi
        _, processed_frame = self.line_detector.detect_red_lines(fused_frame)

        self.update_label_with_image(self.downward_camera_label, processed_frame)

    def update_label_with_image(self, label, cv_image):
        if cv_image is None:
            label.setText("Görüntü alınamıyor...")
            return
        height, width, channel = cv_image.shape
        bytes_per_line = channel * width
        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_image)
        label.setPixmap(pixmap)

if __name__ == "__main__":
    rospy.init_node("line_tracking_gui", anonymous=True)

    app = QApplication(sys.argv)
    gui = FusionGUI()
    gui.show()
    sys.exit(app.exec())
