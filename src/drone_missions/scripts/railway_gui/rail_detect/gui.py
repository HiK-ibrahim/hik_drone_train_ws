import sys
import rospy
import cv2
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QGridLayout
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, Qt
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from füzyon_matrixVeRenkli_Modüler import process_fusion
from drone_control import DroneControl
# Ray detection modülünü import ediyoruz
from ray_and_distance_detect import detect_rails_and_measure

class FusionGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Füzyon Arayüzü")
        self.setGeometry(100, 100, 1600, 800)

        # QLabel'ler
        self.fusion_label = QLabel("Füzyon görüntüsü bekleniyor...")
        self.fusion_label.setAlignment(Qt.AlignCenter)

        self.rail_detection_label = QLabel("Ray tespit görüntüsü bekleniyor...")
        self.rail_detection_label.setAlignment(Qt.AlignCenter)

        self.status_label = QLabel("Durum: Bekleniyor...")
        self.status_label.setAlignment(Qt.AlignCenter)

        # Drone kontrolü için sınıf
        self.drone_control = DroneControl()

        # ROS bağlantıları
        self.bridge = CvBridge()
        self.last_lidar_msg = None
        self.last_front_camera_msg = None
        self.last_lidar_msg_downward = None
        self.last_downward_camera_frame = None

        # ROS Subscribers
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/front_cam/camera/image", Image, self.front_camera_callback)
        rospy.Subscriber("/scan/downward", LaserScan, self.downward_lidar_callback)
        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.downward_camera_callback)

        # Görüntü güncelleme için QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_views)
        self.timer.start(100)  # 100 ms

        # Layout ayarı
        image_layout = QHBoxLayout()
        image_layout.addWidget(self.fusion_label)
        image_layout.addWidget(self.rail_detection_label)

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
        main_layout.addLayout(image_layout)
        main_layout.addWidget(self.status_label)
        main_layout.addLayout(grid_layout)
        self.setLayout(main_layout)

    def lidar_callback(self, msg):
        self.last_lidar_msg = msg

    def front_camera_callback(self, msg):
        self.last_front_camera_msg = msg

    def downward_lidar_callback(self, msg):
        self.last_lidar_msg_downward = msg

    def downward_camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_downward_camera_frame = frame
        except Exception as e:
            rospy.logerr(f"Kamera mesajı işlenirken hata oluştu: {e}")

    def update_views(self):
        if self.last_lidar_msg and self.last_front_camera_msg:
            self.update_fusion_view()

        if self.last_lidar_msg_downward and self.last_downward_camera_frame is not None:
            self.update_rail_detection_view()

    def update_fusion_view(self):
        lidar_mask = process_fusion(self.last_lidar_msg, None)
        front_camera = self.bridge.imgmsg_to_cv2(self.last_front_camera_msg, "bgr8")
        combined_image = self.overlay_lidar_on_camera(lidar_mask, front_camera)
        self.update_label_with_image(self.fusion_label, combined_image)

    def update_rail_detection_view(self):
        # Ray detection fonksiyonunu çağır ve görüntüyü al
        processed_image = detect_rails_and_measure(self.last_lidar_msg_downward, self.last_downward_camera_frame)
        if processed_image is not None:
            self.update_label_with_image(self.rail_detection_label, processed_image)

    def overlay_lidar_on_camera(self, lidar_mask, camera_image):
        if len(lidar_mask.shape) == 2:
            lidar_mask = cv2.cvtColor(lidar_mask, cv2.COLOR_GRAY2BGR)
        camera_image = cv2.resize(camera_image, (lidar_mask.shape[1], lidar_mask.shape[0]))
        alpha = 0.5
        beta = 1 - alpha
        fused_image = cv2.addWeighted(camera_image, beta, lidar_mask, alpha, 0)
        return fused_image

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
    rospy.init_node("fusion_gui", anonymous=True)
    app = QApplication(sys.argv)
    gui = FusionGUI()
    gui.show()
    sys.exit(app.exec())