import asyncio.sslproto

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)
mtx = np.array([[1.27358341e+03, 0.00000000e+00, 3.00942489e+02],
                             [0.00000000e+00, 1.26663343e+03, 2.33213114e+02],
                             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dst = np.array([[7.43051630e-02, -5.16983657e+00, -1.01402024e-03, -2.80294514e-04, 9.13089594e+01]])

markerCorners = None
markerIds = None
rejectedCandidates = None

def euler_from_quaternion(x, y, z, w):
    """
    Преобразует кватернион в углы Эйлера (крен, тангаж, рыскание)
    Крен - вращение вокруг оси x в радианах (положительный поворот против часовой стрелки)
    Тангаж - вращение вокруг оси y в радианах (положительный поворот против часовой стрелки)
    Рыскание - вращение вокруг оси z в радианах (положительный поворот против часовой стрелки)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
            
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
            
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
            
    return roll_x, pitch_y, yaw_z # в радианах
 

def ArucoDetector(frame):
    global mtx, dst, aruco_dictionary, parameters, detector, markerCorners, markerIds, rejectedCandidates

    aruco_side_length = 0.2
    detected = False
    translation_x, translation_z, pitch_y = 0, 0, 0

    # Обнаружение маркеров на кадре
    corners, marker_ids, rejectedImgPoints = detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates)

    # Проверка, что хотя бы один маркер был обнаружен
    if marker_ids is not None:
        detected = True
        # Отрисовка квадрата вокруг обнаруженных маркеров на кадре
        cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
        marker_ids.flatten()
        cv2.imwrite("bazaaaaa.png", frame)

        # Получение векторов поворота и трансляции
        # rvecs, tvecs, _ = cv2.estimatePoseSingleMarkers(corners, aruco_side_length, mtx, dst)

        # Преобразование координат маркеров в трехмерные координаты
        object_points = np.array([[0, 0, 0],
                                  [aruco_side_length, 0, 0],
                                  [0, aruco_side_length, 0],
                                  [aruco_side_length, aruco_side_length, 0]], dtype=np.float32)

        # Получение матрицы камеры и коэффициентов дисторсии
        camera_matrix = np.array(mtx)
        dist_coeffs = np.array(dst)

        # Нахождение векторов поворота и трансляции с помощью solvePnP
        success, rvecs, tvecs = cv2.solvePnP(object_points, corners[0], camera_matrix, dist_coeffs)

        if success:
            translation_x, translation_z, _ = tvecs
        # Печать положения маркера относительно камеры
        # СК камеры: ты - камера и смотришь на мир сквозь объектив :(
        # x-ось указывает направо
        # y-ось указывает вниз
        # z-ось указывает вперед, изображая направление обзора камеры

        # Сохранение информации о трансляции (положении)
        translation_x = tvecs[0]
        translation_y = tvecs[1]
        translation_z = tvecs[2]
 
        # Сохранение информации о вращении
        rotation_matrix = np.eye(4)
        if rvecs.shape == (3, 1):  # Проверяем размерность
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat()
        else:
            print("Неправильные размеры вектора поворота rvecs")
                 
        # Формат кватерниона         
        rotation_x = quat[0] 
        rotation_y = quat[1] 
        rotation_z = quat[2] 
        rotation_w = quat[3] 
                 
        # Углы Эйлера в радианах
        roll_x, pitch_y, yaw_z = euler_from_quaternion(rotation_x,
                                                       rotation_y,
                                                       rotation_z,
                                                       rotation_w)
                 
        roll_x_deg = math.degrees(roll_x)
        pitch_y_deg = math.degrees(pitch_y)
        yaw_z_deg = math.degrees(yaw_z)
        # print("translation_x: {}".format(translation_x))
        # print("translation_y: {}".format(translation_y))
        # print("translation_z: {}".format(translation_z))
        # print("roll_x: {}".format(roll_x_deg))
        # print("pitch_y: {}".format(pitch_y_deg))
        # print("yaw_z: {}".format(yaw_z_deg))
        # print("markers found: {}".format(marker_ids))
        # print()

    return detected, translation_x - math.sin(pitch_y), translation_z - math.cos(pitch_y), pitch_y, marker_ids