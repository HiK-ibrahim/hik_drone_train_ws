#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist  # Hareket komutları için

from line_detect import LineDetector

class LineFollower:
    def __init__(self):
        # LineDetector nesnesi
        self.detector = LineDetector(
            lower_red1=np.array([0, 50, 50]),
            upper_red1=np.array([10, 255, 255]),
            lower_red2=np.array([170, 50, 50]),
            upper_red2=np.array([180, 255, 255])
        )

        # Hareket komutları için publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def follow_line(self, frame):
        # Çizgiyi algıla
        mask, processed_frame = self.detector.detect_red_lines(frame)

        # Maskeden kontur bilgisi al
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # En büyük konturu al
            largest_contour = max(contours, key=cv2.contourArea)
            moment = cv2.moments(largest_contour)

            # Konturun merkezi
            if moment["m00"] > 0:
                cx = int(moment["m10"] / moment["m00"])
                cy = int(moment["m01"] / moment["m00"])

                # Merkez noktayı çiz
                cv2.circle(processed_frame, (cx, cy), 5, (0, 255, 0), -1)

                # Çizginin eğimini hesapla
                angle = self.calculate_line_angle(largest_contour)

                # Yön bilgisi oluştur
                direction = self.get_direction(cx, frame.shape[1], angle)
                self.send_movement_command(direction)
                return processed_frame, direction

        # Çizgi bulunamazsa drone'u durdur
        self.stop_drone()
        return processed_frame, "Line Not Found"

    def calculate_line_angle(self, contour):
        """
        Konturdaki en uygun doğruyu (fit line) bul ve eğimini hesapla.
        """
        [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = np.arctan2(vy, vx) * 180 / np.pi  # Radyanı dereceye çevir
        return angle

    def get_direction(self, cx, width, angle):
        """
        Çizginin merkezine ve eğimine göre hareket yönünü belirler.
        """
        if cx < width // 3:
            return "Turn Left"
        elif cx > 2 * width // 3:
            return "Turn Right"
        elif -10 < angle < 10:
            return "Go Straight"
        elif angle > 10:
            return "Adjust Left"
        elif angle < -10:
            return "Adjust Right"
        else:
            return "Line Not Found"

    def send_movement_command(self, direction):
        """
        Hareket yönüne göre drone'a hareket komutu gönderir.
        """
        cmd = Twist()
        if direction == "Turn Left":
            cmd.angular.z = 0.5  # Saat yönünün tersine dönüş
            cmd.linear.x = 0.2   # Hafif ileri hareket
        elif direction == "Turn Right":
            cmd.angular.z = -0.5  # Saat yönünde dönüş
            cmd.linear.x = 0.2    # Hafif ileri hareket
        elif direction == "Go Straight":
            cmd.linear.x = 0.5  # İleri hareket
        elif direction == "Adjust Left":
            cmd.angular.z = 0.3  # Daha küçük bir dönüş açısı
            cmd.linear.x = 0.3
        elif direction == "Adjust Right":
            cmd.angular.z = -0.3
            cmd.linear.x = 0.3

        # Komutu yayınla
        self.cmd_vel_pub.publish(cmd)

    def stop_drone(self):
        """
        Drone'u durdurmak için komut gönderir.
        """
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)  # Tüm hızları sıfırla