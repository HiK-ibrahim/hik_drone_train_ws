import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

# Global değişkenler
bridge = CvBridge()
last_lidar_msg_downward = None
last_camera_frame = None

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

def get_transform_matrix():
    # Translation (x, y, z)
    translation = [0.000, -0.000, -0.053]  # Alt lidar ile alt kamera arası
    quaternion = [-0.500, -0.500, -0.500, 0.500]
    rot_matrix = quaternion_matrix(quaternion)
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 3] = translation
    transform_matrix[0:3, 0:3] = rot_matrix[0:3, 0:3]
    return transform_matrix

def detect_rails_and_measure(scan):
    if last_camera_frame is None:
        rospy.logwarn("Kamera görüntüsü bekleniyor...")
        return

    # Kamera görüntüsünü lidar katmanı için temel olarak kullan
    lidar_layer = last_camera_frame.copy()

    # Kameranın FOV'üne göre lidar verilerini sınırlıyoruz
    half_fov = hfov_rad / 2  # Kameranın sağ ve sol görüş açısının yarısı

    transform_matrix = get_transform_matrix()
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    rail_points = []

    for i, r in enumerate(scan.ranges):
        if scan.range_min < r < scan.range_max:
            angle = angle_min + i * angle_increment

            # Lidar verisinin açısı kameranın FOV'ü içinde mi?
            if -half_fov <= angle <= half_fov:
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
                        rail_points.append((x, y, z))
                        # Ray noktalarını kırmızı ile işaretle
                        cv2.circle(lidar_layer, (pixel[0], pixel[1]), 3, (0, 0, 255), -1)

    # Ray tespiti ve yükseklik analizi
    if rail_points:
        rail_z_values = [point[2] for point in rail_points]
        ground_level = np.percentile(rail_z_values, 1)  # Zemin seviyesi için alt %10
        rail_heights = [z - ground_level for z in rail_z_values]
        avg_height = np.mean(rail_heights)

        # Zemin referansı noktalarını mavi ile işaretle
        for point in rail_points:
            if abs(point[2] - ground_level) < 0.05:  # Zemine yakın noktaları seç
                ground_pixel = camera_matrix @ np.array([point[0], point[1], point[2]])
                ground_pixel = (ground_pixel / ground_pixel[2]).astype(int)
                if 0 <= ground_pixel[0] < res_x and 0 <= ground_pixel[1] < res_y:
                    cv2.circle(lidar_layer, (ground_pixel[0], ground_pixel[1]), 5, (255, 0, 0), -1)

        # Yükseklik bilgisini ekrana yazdır
        rospy.loginfo(f"Ray zeminden ortalama yükseklik: {avg_height:.2f} metre")
        cv2.putText(lidar_layer, f"Avg Height: {avg_height:.2f} m", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Görselleştirme penceresi
    cv2.imshow("Ray Detection and Height Visualization", lidar_layer)
    cv2.waitKey(1)


def lidar_callback(scan):
    global last_lidar_msg_downward
    last_lidar_msg_downward = scan
    detect_rails_and_measure(scan)

def camera_callback(msg):
    global last_camera_frame
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        last_camera_frame = cv2.resize(frame, (res_x, res_y))
    except Exception as e:
        rospy.logerr(f"Kamera mesajı işlenirken hata oluştu: {e}")

if __name__ == "__main__":
    rospy.init_node("ray_and_distance_detection")
    rospy.Subscriber("/scan/downward", LaserScan, lidar_callback)
    rospy.Subscriber("/downward_cam/downward_camera/image", Image, camera_callback)

    rospy.spin()

    cv2.destroyAllWindows()
