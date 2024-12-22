import rospy
from geometry_msgs.msg import Twist

class DroneControl:
    def __init__(self):
        # ROS Publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def set_text(self, label, text):
        """
        Arayüzdeki bir QLabel'i günceller.
        """
        label.setText(f"Durum: {text}")

    def up_fun(self, label):
        self.set_text(label, "Up")
        vel_msg = Twist()
        vel_msg.linear.z = 1.0
        self.vel_pub.publish(vel_msg)

    def down_fun(self, label):
        self.set_text(label, "Down")
        vel_msg = Twist()
        vel_msg.linear.z = -1.0
        self.vel_pub.publish(vel_msg)

    def forward_fun(self, label):
        self.set_text(label, "Forward")
        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        self.vel_pub.publish(vel_msg)

    def backward_fun(self, label):
        self.set_text(label, "Backward")
        vel_msg = Twist()
        vel_msg.linear.x = -1.0  # Hata düzeltiliyor: burada vel_pub yerine vel_msg olmalı
        self.vel_pub.publish(vel_msg)

    def right_fun(self, label):
        self.set_text(label, "Right")
        vel_msg = Twist()
        vel_msg.linear.y = -1.0
        self.vel_pub.publish(vel_msg)

    def left_fun(self, label):
        self.set_text(label, "Left")
        vel_msg = Twist()
        vel_msg.linear.y = 1.0
        self.vel_pub.publish(vel_msg)

    def cw_fun(self, label):
        self.set_text(label, "Turn Right")
        vel_msg = Twist()
        vel_msg.angular.z = -1.0
        self.vel_pub.publish(vel_msg)

    def ccw_fun(self, label):
        self.set_text(label, "Turn Left")
        vel_msg = Twist()
        vel_msg.angular.z = 1.0
        self.vel_pub.publish(vel_msg)

    def enviar_velocidad(self, vx, vy, vz, vaz):
        """
        Drone'a hız komutları gönderir.
        """
        vel_msg = Twist()
        vel_msg.linear.x = float(vx)
        vel_msg.linear.y = float(vy)
        vel_msg.linear.z = float(vz)
        vel_msg.angular.z = float(vaz)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        self.vel_pub.publish(vel_msg)

    def hover_pub(self, label):
        """
        Drone'u sabit tutar.
        """
        self.set_text(label, "Hover")
        self.enviar_velocidad(0.0, 0.0, 0.0, 0.0)
