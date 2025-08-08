#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import os
import glob
import sys
import termios
import tty
import select

SAVE_DIRS = {
    'a': 'dataset/raw/classA',
    'b': 'dataset/raw/classB'
}

save_counts = {
    'a': 0,
    'b': 0
}

bridge = CvBridge()
current_image = None

def ensure_dirs():
    for path in SAVE_DIRS.values():
        os.makedirs(path, exist_ok=True)

def get_next_index(directory, prefix):
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
    if current_image is None:
        print("Нет изображения для сохранения.")
        return
    dir_path = SAVE_DIRS[class_key]
    prefix = os.path.basename(dir_path)
    idx = get_next_index(dir_path, prefix)
    filename = f"{prefix}_{idx:03d}.jpg"
    filepath = os.path.join(dir_path, filename)
    cv2.imwrite(filepath, current_image)
    save_counts[class_key] += 1
    print(f"[{class_key.upper()}] Сохранено: {save_counts[class_key]} | Путь: {filepath}")

def get_key(timeout=0.1):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    global current_image

    rospy.init_node('image_saver_node', anonymous=True)
    ensure_dirs()
    rospy.Subscriber("/front_camera/image_raw/compressed", CompressedImage, image_callback)
    print("Готов к приёму изображений. Нажмите 'a' или 'b' для сохранения, 'q' — выход.")

    # Настройка терминала для чтения без Enter
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
                print("Завершение работы.")
                break
            rospy.sleep(0.05)
    except KeyboardInterrupt:
        print("Принудительное завершение.")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    import cv2  # Импорт в main, чтобы не падало при ошибках GUI
    main()
