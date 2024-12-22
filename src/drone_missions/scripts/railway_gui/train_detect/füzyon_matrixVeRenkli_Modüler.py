import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

# Global değişkenler
bridge = CvBridge()
last_lidar_msg = None

# Kamera parametreleri
res_x = 640
res_y = 480
hfov_deg = 90

hfov_rad = np.deg2rad(hfov_deg)
f_x = res_x / (2 * np.tan(hfov_rad / 2))
f_y = f_x
c_x = res_x / 2
c_y = res_y / 2

camera_matrix = np.array([
    [f_x, 0, c_x],
    [0, f_y, c_y],
    [0, 0, 1]
])

# LIDAR'dan Kameraya dönüşüm matrisi
def get_transform_matrix():
    # Translation (x, y, z)
    translation = [0.000, -0.153, -0.050]

    # Rotation (quaternion)
    quaternion = [0.500, 0.500, 0.500, -0.500]
    rot_matrix = quaternion_matrix(quaternion)

    # Homojen dönüşüm matrisi
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 3] = translation
    transform_matrix[0:3, 0:3] = rot_matrix[0:3, 0:3]
    return transform_matrix

def create_lidar_layer(scan):
    lidar_layer = np.zeros((res_y, res_x, 3), dtype=np.uint8)  # Siyah arka plan
    transform_matrix = get_transform_matrix()

    angle_min = scan.angle_min
    angle_increment = scan.angle_increment

    for i, r in enumerate(scan.ranges):
        if scan.range_min < r < scan.range_max:
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = 0.0
            lidar_point = np.array([x, y, z, 1]).reshape(4, 1)
            camera_point = transform_matrix @ lidar_point
            x, y, z = camera_point[:3].flatten()
            if z > 0:
                pixel = camera_matrix @ np.array([x, y, z])
                pixel = (pixel / pixel[2]).astype(int)
                if 0 <= pixel[0] < res_x and 0 <= pixel[1] < res_y:
                    # Renk seçimi: Mesafeye göre renk kodlaması
                    if r < 1.0:  # Çok yakın mesafe
                        color = (0, 0, 255)  # Kırmızı
                    elif r < 2.5:  # Orta mesafe
                        color = (0, 255, 255)  # Sarı
                    else:  # Uzak mesafe
                        color = (0, 255, 0)  # Yeşil

                    cv2.circle(lidar_layer, (pixel[0], pixel[1]), 3, color, -1)  # Nokta çizimi
    return lidar_layer

def create_camera_layer(image_msg):
    if image_msg is None:
        # Gelen mesaj yoksa boş bir görüntü döndür (hiçbir şey loglama)
        return np.zeros((res_y, res_x, 3), dtype=np.uint8)

    try:
        return bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
    except Exception:
        # Hata durumunda da loglama yapma
        return np.zeros((res_y, res_x, 3), dtype=np.uint8)


def combine_layers(camera_layer, lidar_layer):
    return cv2.addWeighted(camera_layer, 0.7, lidar_layer, 0.3, 0)  # Ağırlıklı birleştirme

def camera_callback(image_msg):
    global last_lidar_msg
    if last_lidar_msg is None:
        return

    camera_layer = create_camera_layer(image_msg)
    lidar_layer = create_lidar_layer(last_lidar_msg)

    # Katmanları birleştir
    combined_layer = combine_layers(camera_layer, lidar_layer)

    # Görüntüyü ekrana bas
    cv2.imshow("Combined View", combined_layer)
    cv2.waitKey(1)

def process_fusion(last_lidar_msg, image_msg):
    if last_lidar_msg is None:
        return np.zeros((res_y, res_x, 3), dtype=np.uint8)

    camera_layer = create_camera_layer(image_msg)
    lidar_layer = create_lidar_layer(last_lidar_msg)

    return combine_layers(camera_layer, lidar_layer)


if __name__ == "__main__":
    rospy.init_node("lidar_camera_fusion_layers")

    rospy.Subscriber("/scan", LaserScan, lambda msg: globals().update(last_lidar_msg=msg))
    rospy.Subscriber("/front_cam/camera/image", Image, camera_callback)

    rospy.spin()
    cv2.destroyAllWindows()
