from typing import final
import numpy as np
import cv2

resolution = (1280, 720)
resolution = (1920, 1080)

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result


resolution = (1280, 720)

corners_points = np.float32([[801, 337], [908, 405],
                             [737, 446], [847, 512]])
#corners_points = corners_points*1.55
#print(corners_points)

field_square = np.float32([[0, 0], [700, 0],
                           [0, 700], [700, 700]])

frame_raw = cv2.imread("images/Aruco2.jpg")

frame_raw = cv2.resize(frame_raw, (100, 100), interpolation=cv2.INTER_LANCZOS4)


frame_raw = cv2.resize(frame_raw, resolution, interpolation=cv2.INTER_LANCZOS4)

transform_matrix = cv2.getPerspectiveTransform(corners_points, field_square)
transformed_frame = cv2.warpPerspective(frame_raw, transform_matrix, (700, 700))
transformed_frame = rotate_image(transformed_frame, 0)
final_frame = cv2.resize(transformed_frame, (60, 60), interpolation=cv2.INTER_LANCZOS4)

cv2.imshow("Frame", frame_raw)
cv2.imshow("Transformed", transformed_frame)
cv2.imshow("Final", final_frame)

aruco_params = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

corners, ids, rejected = aruco_detector.detectMarkers(final_frame)

marker_size = 20
marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

fx = 20.0  # Focal length in pixels (x-axis)
fy = 20.0  # Focal length in pixels (y-axis)
cx = 10.0  # Principal point (x-coordinate)
cy = 10.0  # Principal point (y-coordinate)
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)


dist_coeffs = np.zeros((1, 5), dtype=np.float32)

_, rvec, tvec = cv2.solvePnP(marker_points, corners[0], camera_matrix, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)

R, _ = cv2.Rodrigues(rvec)
yav = np.degrees(np.arctan2(R[1, 0], R[0, 0]))

print(rvec, R ,yav)
print(corners, ids)
#corners = corners.astype(int)
print(corners[0])
rect = cv2.boundingRect(corners)

print(rect)

cv2.waitKey(0)
cv2.destroyAllWindows()