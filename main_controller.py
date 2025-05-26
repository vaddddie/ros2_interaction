import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
import numpy as np
import json

import matplotlib.pyplot as plt

border_size = 0.1

def read_json(path='/home/ubun/main/coppeliasim/mask_rcnn_venv/Mask_RCNN/samples/nail/test_nail/val/via_region_data.json'):
    with open(path, 'r') as file:
        data = json.load(file)

    x_array = data['_via_img_metadata']['output_image.png39943']['regions'][0]['shape_attributes']['all_points_x']
    y_array = data['_via_img_metadata']['output_image.png39943']['regions'][0]['shape_attributes']['all_points_y']

    return x_array, y_array

def create_path(image, depth, sens_pos):
    pixel_step = border_size / len(image)

    path_x = []
    path_y = []
    path_z = []

    x_coords, y_coords = read_json()

    for px_x, px_y in zip(x_coords, y_coords):
        path_x.append(sens_pos.x - (len(image)/2 - 1)*pixel_step + (px_x - 2)*pixel_step)
        path_y.append(sens_pos.y - (len(image)/2 - 1)*pixel_step + (px_y + 2)*pixel_step)
        path_z.append(sens_pos.z - depth[px_y][px_x + 1] + 0.01)

    # path_x.append(sens_pos.x - (len(image)/2 - 1)*pixel_step + x_coords[0]*pixel_step)
    # path_y.append(sens_pos.y - (len(image)/2 - 1)*pixel_step + y_coords[0]*pixel_step)
    # path_z.append(sens_pos.z - depth[y_coords[0]][x_coords[0] + 1] + 0.01)

    return path_x, path_y, path_z


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        self.image = None
        self.depth = None
        self.sensor_pos = None

        self.make_schedule = False

        self.log_target_position = []
        self.log_tip_position = []

        qos_profile = rclpy.qos.QoSProfile(depth=50)
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        self.target_pos_pub = self.create_publisher(
            Vector3,
            'targetPosition',
            qos_profile
        )
        
        self.sensor_pos_sub = self.create_subscription(
            Vector3,  # Тип сообщения
            '/sensor_pos',  # Имя топика
            self.sensor_position_listener,
	    10 # Размер очереди
        )
        
        self.image_sub = self.create_subscription(
            Image,  # Тип сообщения
            '/image',  # Имя топика
            self.image_listener,
	    10 # Размер очереди
        )

        self.depth_sub = self.create_subscription(
            Image,  # Тип сообщения
            '/depth',  # Имя топика
            self.depth_listener,
	    10 # Размер очереди
        )

        self.tip_pos_sub = self.create_subscription(
            Vector3,  # Тип сообщения
            '/tipPosition',  # Имя топика
            self.tip_position_listener,
	    50 # Размер очереди
        )

        self.bridge = CvBridge()  # Инициализация CvBridge

        self.timer = self.create_timer(5.0, self.timer_callback)  # Публикация каждую секунду

    def timer_callback(self):
        if self.image is not None and self.depth is not None and self.sensor_pos is not None:
            self.create_path()
            self.timer.cancel()  # Останавливаем таймер
            self.timer = None  # Убираем ссылку на таймер
            

    def sensor_position_listener(self, msg):
        self.sensor_pos = msg

    def image_listener(self, msg):
        # Декодируем массив как изображение
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image = cv_image

        self.draw_image(cv_image, 'image')

    def depth_listener(self, msg):
        # Декодируем массив как изображение
        depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth = depth_data

        cv_image = np.array([px * 100 for px in depth_data], dtype=np.uint8)
        self.draw_image(cv_image, 'depth')

    def create_path(self):
        x_path, y_path, z_path = create_path(self.image, self.depth, self.sensor_pos)
        for x, y, z in zip(x_path, y_path, z_path):
            msg = Vector3()
            msg.x = x + 0.03 
            msg.y = y
            msg.z = z + 0.1

            self.log_target_position.append(msg)
            
            self.target_pos_pub.publish(msg)

        self.make_schedule = True

    def tip_position_listener(self, msg):
        self.log_tip_position.append(msg)

        if self.make_schedule and len(self.log_tip_position) == len(self.log_target_position):
            err = []
            for i in range(len(self.log_tip_position)):
                err_x = abs(self.log_tip_position[i].x - self.log_target_position[i].x)
                err_y = abs(self.log_tip_position[i].y - self.log_target_position[i].y)
                err_z = abs(self.log_tip_position[i].z - self.log_target_position[i].z)
                err.append(err_x + err_y + err_z)

            plt.plot(np.arange(len(err)), err, marker="o", linestyle="-", color="b")
            plt.title("Ошибка по положению")
            plt.xlabel("Время в с.")
            plt.ylabel("Ошибка положения в cм.")
            plt.legend()

            plt.grid()
            plt.show()

    def draw_image(self, img, title):
        cv2.imshow(title, img)  # Показывает изображение
        cv2.waitKey(0)  # Ждет нажатия клавиши
        cv2.destroyAllWindows()  # Закрывает все окна
        cv2.imwrite('output_image.png', img)  # Сохраняет изображение


def main(args=None):
    # import datetime
    # import time

    # print(f'[{datetime.datetime.now()}] Processing has begun')

    # time.sleep(6.31321)

    # print(f'[{datetime.datetime.now()}] Processing completed')
    
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)

    # По завершении
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
