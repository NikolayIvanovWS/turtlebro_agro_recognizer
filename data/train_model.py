#!/usr/bin/env python3

import os
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras import layers, models
from tensorflow.keras.optimizers import Adam

# Основные параметры
IMAGE_SIZE = (224, 224)
BATCH_SIZE = 16
EPOCHS = 12
DATASET_DIR = 'dataset/processed'  # Папка с подготовленными данными
MODEL_DIR = 'model'
MODEL_H5_PATH = os.path.join(MODEL_DIR, 'model.h5')
MODEL_TFLITE_PATH = os.path.join(MODEL_DIR, 'model.tflite')
LABELS_PATH = os.path.join(MODEL_DIR, 'labels.txt')

os.makedirs(MODEL_DIR, exist_ok=True)

# Аугментация и нормализация для обучающей выборки
train_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=20,
    width_shift_range=0.1,
    height_shift_range=0.1,
    shear_range=0.1,
    zoom_range=0.1,
    horizontal_flip=True,
    fill_mode='nearest'
)

# Нормализация для валидационной выборки
val_datagen = ImageDataGenerator(rescale=1./255)

# Создаем генераторы для train и val из папки dataset/processed
train_generator = train_datagen.flow_from_directory(
    os.path.join(DATASET_DIR, 'train'),
    target_size=IMAGE_SIZE,
    batch_size=BATCH_SIZE,
    class_mode='categorical'
)

val_generator = val_datagen.flow_from_directory(
    os.path.join(DATASET_DIR, 'val'),
    target_size=IMAGE_SIZE,
    batch_size=BATCH_SIZE,
    class_mode='categorical'
)

# Сохраняем имена классов для последующего использования
class_names = list(train_generator.class_indices.keys())
print(f"Классы: {class_names}")

with open(LABELS_PATH, 'w') as f:
    for class_name in class_names:
        f.write(f"{class_name}\n")

# Загружаем предобученную модель без верхних слоев
base_model = MobileNetV2(weights='imagenet', include_top=False, input_shape=(224, 224, 3))
base_model.trainable = False  # Замораживаем веса базовой модели

# Добавляем новые слои классификации
model = models.Sequential([
    base_model,
    layers.GlobalAveragePooling2D(),
    layers.Dense(128, activation='relu'),
    layers.Dropout(0.3),
    layers.Dense(len(class_names), activation='softmax')
])

# Компилируем модель
model.compile(optimizer=Adam(),
              loss='categorical_crossentropy',
              metrics=['accuracy'])

# Обучаем модель
history = model.fit(
    train_generator,
    epochs=EPOCHS,
    validation_data=val_generator
)

# Выводим итоговые метрики
train_acc = history.history['accuracy'][-1]
val_acc = history.history['val_accuracy'][-1]
train_loss = history.history['loss'][-1]
val_loss = history.history['val_loss'][-1]

print(f"\nTrain Accuracy: {train_acc:.4f}, Loss: {train_loss:.4f}")
print(f"Val Accuracy: {val_acc:.4f}, Loss: {val_loss:.4f}")

# Сохраняем модель в формате Keras
model.save(MODEL_H5_PATH)
print(f"Сохранена модель: {MODEL_H5_PATH}")

# Конвертируем модель в формат TensorFlow Lite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

with open(MODEL_TFLITE_PATH, 'wb') as f:
    f.write(tflite_model)

print(f"Сохранена TFLite модель: {MODEL_TFLITE_PATH}")
print(f"Сохранены метки классов: {LABELS_PATH}")
