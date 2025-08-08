#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import numpy as np
import cv2
import sys
import signal
import rospkg
import os

# Попытка импортировать tflite_runtime.Interpreter, fallback на tensorflow.lite.Interpreter
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        rospy.logerr("Neither tflite_runtime nor tensorflow.lite is available.")
        sys.exit(1)

INPUT_SIZE = 224  # Размер входного изображения модели

class TFLiteROSNode:
    def __init__(self):
        rospy.init_node('tflite_inference_node', anonymous=True)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('turtlebro_agro_recognizer')

        # Абсолютные пути к файлам модели и меток
        model_path = os.path.join(pkg_path, 'data', 'model', 'model.tflite')
        labels_path = os.path.join(pkg_path, 'data', 'model', 'labels.txt')

        if not os.path.isfile(model_path):
            rospy.logerr(f"Model file not found: {model_path}")
            sys.exit(1)
        if not os.path.isfile(labels_path):
            rospy.logerr(f"Labels file not found: {labels_path}")
            sys.exit(1)

        self.labels = self.load_labels(labels_path)

        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Подписка на сжатые изображения камеры
        self.sub = rospy.Subscriber('/front_camera/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)
        # Публикация распознанного класса растения
        self.pub = rospy.Publisher('/type_of_plant', String, queue_size=10)

        rospy.loginfo("TFLite ROS node started, waiting for images...")

    def load_labels(self, path):
        """Загрузка списка меток из файла."""
        with open(path, 'r') as f:
            return [line.strip() for line in f.readlines()]

    def preprocess(self, msg):
        """Декодирование и предобработка входного изображения для модели."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            rospy.logwarn("Failed to decode image")
            return None
        img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)
        return img

    def inference(self, input_data):
        """Запуск инференса модели и получение результата с максимальной уверенностью."""
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        top_idx = np.argmax(output_data)
        confidence = output_data[top_idx]
        label = self.labels[top_idx] if top_idx < len(self.labels) else 'Unknown'
        return label, confidence

    def image_callback(self, msg):
        """Обработка входящих изображений, выполнение классификации и публикация результата."""
        try:
            input_data = self.preprocess(msg)
            if input_data is None:
                return
            label, confidence = self.inference(input_data)
            rospy.loginfo(f"Detected: {label} ({confidence:.3f})")
            self.pub.publish(label)
        except Exception as e:
            rospy.logerr(f"Error during inference: {e}")

def signal_handler(sig, frame):
    """Обработка сигнала завершения для корректного выключения ноды."""
    rospy.loginfo("Shutting down node...")
    rospy.signal_shutdown("SIGINT received")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        node = TFLiteROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
