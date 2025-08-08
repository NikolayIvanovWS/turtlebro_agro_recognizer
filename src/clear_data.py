#!/usr/bin/env python3
import rospy
import rospkg
import os
import shutil

def clear_folder(path):
    """Удаляет все файлы и папки в указанной директории."""
    if os.path.exists(path):
        rospy.loginfo(f"Очищаю папку: {path}")
        for filename in os.listdir(path):
            file_path = os.path.join(path, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                    rospy.loginfo(f"Удалён файл: {file_path}")
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
                    rospy.loginfo(f"Удалена папка: {file_path}")
            except Exception as e:
                rospy.logwarn(f"Не удалось удалить {file_path}: {e}")
    else:
        rospy.logwarn(f"Папка не существует: {path}")

def main():
    rospy.init_node('clear_data_node', anonymous=True)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('turtlebro_agro_recognizer')

    model_dir = os.path.join(pkg_path, 'data', 'model')
    dataset_dir = os.path.join(pkg_path, 'data', 'dataset')

    clear_folder(model_dir)
    clear_folder(dataset_dir)

    rospy.loginfo("Очистка завершена.")

if __name__ == '__main__':
    main()
