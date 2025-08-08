#!/usr/bin/env python3

import os
import cv2
import random
import shutil
from pathlib import Path

# Константы
RAW_DIR = 'dataset/raw'
PROCESSED_DIR = 'dataset/processed'
CLASSES = ['classA', 'classB']
SPLITS = {'train': 0.8, 'val': 0.2}
SEED = 42  # Для воспроизводимости

def verify_image(path):
    """Проверка, что изображение можно открыть с помощью OpenCV"""
    img = cv2.imread(str(path))
    return img is not None

def create_dirs():
    """Создаёт необходимые выходные директории"""
    for split in SPLITS:
        for class_name in CLASSES:
            dir_path = os.path.join(PROCESSED_DIR, split, class_name)
            os.makedirs(dir_path, exist_ok=True)

def get_all_images(class_path):
    """Возвращает список валидных изображений в папке"""
    images = []
    for file in os.listdir(class_path):
        if file.lower().endswith('.jpg'):
            full_path = os.path.join(class_path, file)
            if verify_image(full_path):
                images.append(full_path)
            else:
                print(f'[WARN] Не удалось открыть изображение: {full_path}')
    return images

def split_and_copy_images():
    """Разделяет и копирует изображения по папкам train/val"""
    for class_name in CLASSES:
        class_path = os.path.join(RAW_DIR, class_name)
        images = get_all_images(class_path)
        random.shuffle(images)
        split_idx = int(len(images) * SPLITS['train'])

        train_images = images[:split_idx]
        val_images = images[split_idx:]

        for split_name, split_images in [('train', train_images), ('val', val_images)]:
            dest_dir = os.path.join(PROCESSED_DIR, split_name, class_name)
            for src_path in split_images:
                filename = os.path.basename(src_path)
                dest_path = os.path.join(dest_dir, filename)
                shutil.copy2(src_path, dest_path)

def count_images():
    """Подсчитывает количество изображений в каждой целевой папке"""
    print("\n[INFO] Количество изображений в папках:")
    for split in SPLITS:
        for class_name in CLASSES:
            dir_path = os.path.join(PROCESSED_DIR, split, class_name)
            count = len([f for f in os.listdir(dir_path) if f.lower().endswith('.jpg')])
            print(f'{split}/{class_name}: {count} изображений')

def main():
    print("[INFO] Подготовка датасета...")
    random.seed(SEED)
    create_dirs()
    split_and_copy_images()
    count_images()
    print("\n[INFO] Подготовка завершена.")

if __name__ == '__main__':
    main()
