#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import numpy as np
import cv2
import sys
import signal

# Попытка импортировать tflite_runtime.Interpreter или fallback на tensorflow.lite.Interpreter
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        rospy.logerr("Neither tflite_runtime nor tensorflow.lite is available.")
        sys.exit(1)

MODEL_PATH = 'model/model.tflite'
LABELS_PATH = 'model/labels.txt'
INPUT_SIZE = 224

class TFLiteROSNode:
    def __init__(self):
        rospy.init_node('tflite_inference_node', anonymous=True)

        # Загрузка меток
        self.labels = self.load_labels(LABELS_PATH)

        # Загрузка модели
        self.interpreter = Interpreter(model_path=MODEL_PATH)
        self.interpreter.allocate_tensors()

        # Получение информации о входном и выходном тензоре
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Подписка на изображение
        self.sub = rospy.Subscriber('/front_camera/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)

        # Публикация распознанного класса
        self.pub = rospy.Publisher('/type_of_plant', String, queue_size=10)

        rospy.loginfo("TFLite ROS node started, waiting for images...")

    def load_labels(self, path):
        with open(path, 'r') as f:
            return [line.strip() for line in f.readlines()]

    def preprocess(self, msg):
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
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        top_idx = np.argmax(output_data)
        confidence = output_data[top_idx]
        label = self.labels[top_idx] if top_idx < len(self.labels) else 'Unknown'
        return label, confidence

    def image_callback(self, msg):
        try:
            input_data = self.preprocess(msg)
            if input_data is None:
                return
            label, confidence = self.inference(input_data)
            print(f"Detected: {label} ({confidence:.3f})")

            # Публикация результата
            self.pub.publish(label)
        except Exception as e:
            rospy.logerr(f"Error during inference: {e}")

def signal_handler(sig, frame):
    rospy.loginfo("Shutting down node...")
    rospy.signal_shutdown("SIGINT received")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        node = TFLiteROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
