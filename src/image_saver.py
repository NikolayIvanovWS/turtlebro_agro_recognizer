#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import sys
import termios
import tty
import select
import rospkg

# Папки для сохранения изображений по классам (будут определены динамически)
SAVE_DIRS = {}

# Счётчики сохранённых изображений для каждого класса
save_counts = {
    'a': 0,
    'b': 0
}

bridge = CvBridge()
current_image = None

def ensure_dirs():
    """Создаёт папки для сохранения, если их нет."""
    for path in SAVE_DIRS.values():
        os.makedirs(path, exist_ok=True)

def get_next_index(directory, prefix):
    """Возвращает следующий индекс файла для автонумерации, анализируя существующие файлы."""
    files = glob.glob(os.path.join(directory, f"{prefix}_*.jpg"))
    indices = []
    for f in files:
        try:
            name = os.path.basename(f)
            index = int(name.replace(f"{prefix}_", "").replace(".jpg", ""))
            indices.append(index)
        except ValueError:
            continue
    return max(indices + [0]) + 1

def image_callback(msg):
    """Обработчик входящих изображений, декодирует и масштабирует их."""
    global current_image
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn("Failed to decode image")
            return
        if image.shape[1] != 640 or image.shape[0] != 480:
            image = cv2.resize(image, (640, 480))
        current_image = image
    except Exception as e:
        rospy.logerr(f"Image decode error: {e}")

def save_image(class_key):
    """Сохраняет текущее изображение в папку соответствующего класса с автонумерацией."""
    global save_counts, current_image
    if current_image is None:
        rospy.logwarn("Нет изображения для сохранения.")
        return
    dir_path = SAVE_DIRS[class_key]
    prefix = os.path.basename(dir_path)
    idx = get_next_index(dir_path, prefix)
    filename = f"{prefix}_{idx:03d}.jpg"
    filepath = os.path.join(dir_path, filename)
    cv2.imwrite(filepath, current_image)
    save_counts[class_key] += 1
    rospy.loginfo(f"[{class_key.upper()}] Сохранено: {save_counts[class_key]} | Путь: {filepath}")

def get_key(timeout=0.1):
    """Читает одиночный символ с клавиатуры без необходимости нажимать Enter."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    global current_image

    rospy.init_node('image_saver_node', anonymous=True)

    # Получаем абсолютные пути к папкам сохранения через rospkg
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('turtlebro_agro_recognizer')

    global SAVE_DIRS
    SAVE_DIRS = {
        'a': os.path.join(pkg_path,'data', 'dataset', 'raw', 'classA'),
        'b': os.path.join(pkg_path,'data', 'dataset', 'raw', 'classB')
    }

    ensure_dirs()
    rospy.Subscriber("/front_camera/image_raw/compressed", CompressedImage, image_callback)

    rospy.loginfo("Готов к приёму изображений. Нажмите 'a' или 'b' для сохранения, 'q' — выход.")

    # Настраиваем терминал для чтения клавиш в режиме без ожидания Enter
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'a':
                save_image('a')
            elif key == 'b':
                save_image('b')
            elif key == 'q':
                rospy.loginfo("Завершение работы.")
                break
            rospy.sleep(0.05)
    except KeyboardInterrupt:
        rospy.loginfo("Принудительное завершение.")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()
