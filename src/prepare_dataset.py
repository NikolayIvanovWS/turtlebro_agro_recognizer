#!/usr/bin/env python3

import os
import cv2
import random
import shutil
import rospy
import rospkg

# Константы для путей и классов, переопределяются в main()
RAW_DIR = ''
PROCESSED_DIR = ''
CLASSES = ['classA', 'classB']
SPLITS = {'train': 0.8, 'val': 0.2}
SEED = 42  # Для воспроизводимости рандома

def verify_image(path):
    """Проверяет, что изображение корректно открывается OpenCV."""
    img = cv2.imread(str(path))
    return img is not None

def create_dirs():
    """Создаёт структуру папок для тренировочного и валидационного наборов."""
    for split in SPLITS:
        for class_name in CLASSES:
            dir_path = os.path.join(PROCESSED_DIR, split, class_name)
            os.makedirs(dir_path, exist_ok=True)

def get_all_images(class_path):
    """Возвращает список валидных изображений в заданной папке."""
    images = []
    for file in os.listdir(class_path):
        if file.lower().endswith('.jpg'):
            full_path = os.path.join(class_path, file)
            if verify_image(full_path):
                images.append(full_path)
            else:
                rospy.logwarn(f'[WARN] Не удалось открыть изображение: {full_path}')
    return images

def split_and_copy_images():
    """Разбивает изображения на тренировочную и валидационную выборки, копирует их в нужные папки."""
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
                rospy.loginfo(f'[INFO] Скопировано {src_path} -> {dest_path}')

def count_images():
    """Подсчитывает количество изображений в целевых папках и выводит информацию."""
    rospy.loginfo("\n[INFO] Количество изображений в папках:")
    for split in SPLITS:
        for class_name in CLASSES:
            dir_path = os.path.join(PROCESSED_DIR, split, class_name)
            count = len([f for f in os.listdir(dir_path) if f.lower().endswith('.jpg')])
            rospy.loginfo(f'{split}/{class_name}: {count} изображений')

def main():
    rospy.init_node('prepare_dataset_node', anonymous=True)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('turtlebro_agro_recognizer')

    global RAW_DIR, PROCESSED_DIR
    RAW_DIR = os.path.join(pkg_path, 'data', 'dataset', 'raw')
    PROCESSED_DIR = os.path.join(pkg_path, 'data', 'dataset', 'processed')

    rospy.loginfo("[INFO] Подготовка датасета...")
    random.seed(SEED)
    create_dirs()
    split_and_copy_images()
    count_images()
    rospy.loginfo("[INFO] Подготовка завершена.")

if __name__ == '__main__':
    main()
