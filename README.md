# turtlebro_agro_recognizer

Пакет ROS для сбора, подготовки и классификации изображений растений с использованием Raspberry Pi и TensorFlow Lite. Включает узлы для съёмки изображений, подготовки датасета, инференса модели и утилиту для очистки данных.

---

## Возможности

- Захват изображений с камеры и сохранение по классам (`classA`, `classB`)  
- Подготовка датасета с разбиением на `train` / `val`  
- Классификация растений с помощью модели TensorFlow Lite  
- Утилита для полной очистки папок с моделью и датасетом

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
├── src/
│   ├── image_saver.py
│   ├── prepare_dataset.py
│   ├── plant_classifier_node.py
│   └── clear_data.py
├── data/                 # директория для датасета, модели и скрипта обучения
│   ├── train_model.py
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
- Python-библиотеки: `rospkg`, `numpy`, `cv_bridge`

> Примечание: для экономии ресурсов на Raspberry Pi рекомендуется использовать `tflite-runtime`

---

## Установка

1. Клонируй репозиторий в рабочую область `catkin_ws`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/NikolayIvanovWS/turtlebro_agro_recognizer.git
```

2. Собери workspace:

```bash
cd ~/catkin_ws
catkin_make
```

3. Установи TFLite runtime:

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update
sudo apt install python3-tflite-runtime
```

---

## Запуск


### 1. Захват изображений (image_saver)

Запускает ноду, подписывающуюся на камеру и сохраняющую снимки по классам.

```bash
roslaunch turtlebro_agro_recognizer image_saver.launch
```

Управление в терминале: нажми `a` или `b` для сохранения изображения в соответствующую папку; `q` — выход.

### 2. Подготовка датасета (prepare_dataset)

Разбивает собранные изображения на `train` / `val` и копирует их в `data/dataset/processed`.

```bash
roslaunch turtlebro_agro_recognizer prepare_dataset.launch
```

---

### 3. Обучение модели на компьютере (Windows / Linux)

После подготовки датасета на Raspberry Pi перенеси папку `data` на компьютер и запусти тренировку:

1. **Скопируй подготовленный датасет и скрипт обучения на ПК** (пример для Windows — выполняется на ПК):

```bash
scp -r pi@<IP_АДРЕС_РОБОТА>:/home/pi/catkin_ws/src/turtlebro_agro_recognizer/data "C:\Users\<имя_пользователя>\Documents"
```

2. **Запусти обучение** (на ПК):

```bash
python3 C:\Users\<имя_пользователя>\Documents\data\train_model.py
```

Скрипт использует датасет из `data/dataset/processed` и сохраняет `model.tflite` и `labels.txt` в `data/model/`.

3. **Верни файлы на Raspberry Pi** (выполняется на ПК):

- Скопировать конкретные файлы:

```bash
scp "C:\Users\<имя_пользователя>\Documents\data\model\labels.txt" "C:\Users\<имя_пользователя>\Documents\data\model\model.tflite" pi@<IP_АДРЕС_РОБОТА>:/home/pi/catkin_ws/src/turtlebro_agro_recognizer/data/model/
```

---

### 4. Запуск классификатора (plant_classifier)

После размещения `model.tflite` и `labels.txt` в `data/model/` запусти ноду классификации:

```bash
roslaunch turtlebro_agro_recognizer plant_classifier.launch
```

Нода подписывается на `/front_camera/image_raw/compressed` и публикует распознанный класс в `/type_of_plant`.

---

### 5. Очистка данных (clear_data)

Удаляет содержимое `data/model` и `data/dataset`.

```bash
roslaunch turtlebro_agro_recognizer clear_data.launch
```

---

## Топики

| Топик                                  | Тип                                 | Описание                               |
|----------------------------------------|-------------------------------------|----------------------------------------|
| `/front_camera/image_raw/compressed`   | `sensor_msgs/CompressedImage`       | Входящие сжатые изображения камеры     |
| `/type_of_plant`                       | `std_msgs/String`                   | Определённый класс растения (строка)   |

---
