import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

class RailFollower:
    def __init__(self):
        rospy.init_node("rail_follower", anonymous=True)

        # ROS Subscribers ve Publishers
        self.bridge = CvBridge()
        self.front_camera_sub = rospy.Subscriber("/front_cam/camera/image", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Görüntü işleme için veri
        self.last_frame = None

        # Kontrol parametreleri
        self.speed = 0.8
        self.angular_speed_factor = 0.005

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_frame = frame
        except Exception as e:
            rospy.logerr(f"Kamera mesajı işlenirken hata oluştu: {e}")

    def process_frame(self, frame):
        """
        Tren raylarını tespit etmek için görüntü işleme.
        """
        try:
            # HSV renk uzayına dönüştür
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Metalik gri için HSV aralığı
            lower_gray = np.array([0, 0, 80])
            upper_gray = np.array([180, 50, 200])

            # Maske oluştur
            mask = cv2.inRange(hsv, lower_gray, upper_gray)

            # Maskeyi orijinal görüntüye uygula
            filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)

            # Gri tonlamaya çevir ve bulanıklaştır
            gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Kenar tespiti
            edges = cv2.Canny(blurred, 50, 150)

            # Rayları tespit etmek için ROI ve Hough Lines
            height, width = edges.shape
            roi = edges[int(height / 2):, :]
            lines = cv2.HoughLinesP(roi, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=20)

            # Ray çizgilerini çizin ve merkeze olan mesafeyi hesaplayın
            center_offset = None
            if lines is not None:
                x_coords = []
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    x_coords.extend([x1, x2])
                    cv2.line(frame, (x1, y1 + int(height / 2)), (x2, y2 + int(height / 2)), (0, 255, 0), 2)

                # Yönlendirme için merkeze olan mesafe
                center_offset = (np.mean(x_coords) - width / 2)

            return frame, center_offset
        except Exception as e:
            rospy.logerr(f"Görüntü işleme sırasında hata oluştu: {e}")
            return frame, None

    def follow_rails(self):
        """
        Kameradan gelen görüntüleri işleyerek ray takibi yapar.
        """
        if self.last_frame is not None:
            processed_frame, center_offset = self.process_frame(self.last_frame)

            # Kontrol mesajları
            twist = Twist()
            twist.linear.x = self.speed

            if center_offset is not None:
                twist.angular.z = -center_offset * self.angular_speed_factor

            self.cmd_vel_pub.publish(twist)

            # Görüntüyü göster
            cv2.imshow("Rail Detection", processed_frame)
            cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.follow_rails()
            rate.sleep()

if __name__ == "__main__":
    rail_follower = RailFollower()
    try:
        rail_follower.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
