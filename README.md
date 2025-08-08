# turtlebro_agro_recognizer

Пакет ROS для распознавания растений в агротематике с использованием TensorFlow Lite.  
Включает сбор и подготовку данных, инференс модели на изображениях с камеры, а также утилиту очистки данных.

---

## Возможности

- Захват изображений с камеры и сохранение в разные классы (classA, classB)  
- Подготовка датасета с разбиением на train/val  
- Классификация растений на основе модели TensorFlow Lite  
- Очистка папок с данными и моделью  

---

## Структура пакета

```
turtlebro_agro_recognizer/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── image_saver.launch
│   ├── prepare_dataset.launch
│   ├── plant_classifier.launch
│   └── clear_data.launch
├── scripts/
│   ├── image_saver.py
│   ├── prepare_dataset.py
│   ├── plant_classifier_node.py
│   └── clear_data.py
├── data/
│   ├── model/
│   │   ├── model.tflite
│   │   └── labels.txt
│   └── dataset/
│       ├── raw/
│       │   ├── classA/
│       │   └── classB/
│       └── processed/
│           ├── train/
│           └── val/
└── README.md
```

---

## Требования

- ROS Noetic (Ubuntu 20.04)  
- Python 3  
- OpenCV для Python (`python3-opencv`)  
- TensorFlow Lite runtime или TensorFlow (для инференса модели)  
- rospkg, numpy, cv_bridge

---

## Установка

1. Клонируйте репозиторий в ваш `catkin_ws/src`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/NikolayIvanovWS/turtlebro_agro_recognizer.git
```

2. Соберите workspace:

```bash
cd ~/catkin_ws
catkin_make
```

3. Установите, необходимые Python-зависимости:

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update
sudo apt install python3-tflite-runtime
```

---

## Запуск

### 1. Запись изображений

Запускает подписку на камеру и сохранение снимков по классам.

```bash
roslaunch turtlebro_agro_recognizer image_saver.launch
```

Нажмите `a` или `b` для сохранения изображения в соответствующую папку. `q` — выход.

---

### 2. Подготовка датасета

Разбивает собранные изображения на train/val и копирует их в нужные папки.

```bash
roslaunch turtlebro_agro_recognizer prepare_dataset.launch
```

---

### 3. Запуск классификатора

Подписывается на поток изображений и публикует распознанный класс в топик `/type_of_plant`.

```bash
roslaunch turtlebro_agro_recognizer plant_classifier.launch
```

---

### 4. Очистка данных

Удаляет все содержимое папок `data/model` и `data/dataset`.

```bash
roslaunch turtlebro_agro_recognizer clear_data.launch
```

---

## Топики

| Топик                             | Тип           | Описание                        |
|----------------------------------|---------------|--------------------------------|
| `/front_camera/image_raw/compressed` | sensor_msgs/CompressedImage | Входящие сжатые изображения камеры |
| `/type_of_plant`                  | std_msgs/String | Распознанный класс растения    |

---
