import cv2 as cv
from cv2 import aruco
import numpy as np

def load_image(image_path):
    # Загружаем изображение с указанного пути
    image = cv.imread(image_path)
    if image is None:
        print(f"Ошибка загрузки изображения с {image_path}")
    return image

def detect_aruco_markers(image, camera_matrix=None, dist_coeffs=None):
    # Создаем детектор Aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    # Преобразуем изображение в серый цвет
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Ищем метки Aruco
    corners, ids, rejected = detector.detectMarkers(gray)

    # Если метки найдены, рисуем их на изображении
    if len(corners) > 0:
        aruco.drawDetectedMarkers(image, corners, ids)
        print(f"Найдены метки Aruco с ID: {ids.flatten()}")
    else:
        print("Метки Aruco не найдены")

    # Если заданы параметры камеры, оцениваем позу меток
    if camera_matrix is not None and dist_coeffs is not None:
        for i in range(len(corners)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
            aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            print(f"Метка ID {ids[i]}: rvec = {rvec}, tvec = {tvec}")

    # Показать изображение с найденными метками
    cv.imshow("Aruco Markers", image)
    cv.waitKey(0)
    cv.destroyAllWindows()

def main():
    # Укажите путь к изображению
    image_path = "baza.png"
    
    # Загрузите изображение
    image = load_image(image_path)
    
    # Параметры камеры (например, для тестирования можно оставить None)
    camera_matrix = None  # Замените на вашу матрицу камеры
    dist_coeffs = None    # Замените на ваши коэффициенты дисторсии

    # Найдите и отобразите метки Aruco
    detect_aruco_markers(image, camera_matrix, dist_coeffs)

if __name__ == "__main__":
    main()
