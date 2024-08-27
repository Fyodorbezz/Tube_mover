from re import I
from typing import Self
import numpy as np
import cv2

resolution = (1280, 720)
#resolution = (1920, 1080)
window_origin = (0, 0)
window_name = "Field"
sliders_window = "Coefs"
LAB_image_window = "LAB"
masked_image_window = "Masked"


'''corners_points = np.float32([[300, 49], [930, 67],
                             [77, 692], [1125, 696]])'''

corners_points = np.float32([[338, 33], [992, 32],
                             [141, 701], [1241, 689]])

corners_points = np.float32([[300, 39], [945, 41],
                             [44, 685], [1129, 706]])

corners_points = np.float32([[363, 60], [997, 64],
                             [176, 704], [1264, 696]])

field_square = np.float32([[-2, -5], [702, -5],
                           [-2, 702], [702, 702]])

field_square = np.float32([[0, 0], [1400, 0],
                           [0, 1400], [1400, 1400]])

corners_points = np.float32([[363*1.55, 60*1.55], [997*1.55, 64*1.55],
                             [176*1.55, 704*1.55], [1264*1.55, 696*1.55]])



line_points = [[0, 0], [0, 0]]
point_index = 0

transformed_frame = np.zeros((1,1,3), np.uint8)
LAB_frame = np.zeros((1,1,3), np.uint8)
masked_frame = np.zeros((1,1,3), np.uint8)

lover_LAB = np.array([71, 119, 124])
upper_LAB = np.array([152, 127, 136])
erosion = 2
dilation = 2

erosion_iter = 1
dilation_iter = 1


sliders = list()
sliders_name = ["Low L", "High L", "Low A", "High A", "Low B", "High B"]

pixels_min = [255]*3
pixels_max = [0]*3

def print_mouse_coordinates(event, x, y, flags, param):
    global line_points, point_index, transformed_frame

    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, y)
    if event == cv2.EVENT_RBUTTONDOWN:
        line_points[point_index] = [x, y]
        if point_index == 1:
            cv2.line(transformed_frame, line_points[0], line_points[1], (255, 0, 255), 3)
        point_index += 1
        point_index = point_index%2

def update_binarise_image():
    global masked_frame, lover_LAB, upper_LAB
    lover_LAB = np.array([int(sliders[0].value), int(sliders[2].value), int(sliders[4].value)])
    upper_LAB = np.array([int(sliders[1].value), int(sliders[3].value), int(sliders[5].value)])
    masked_frame = cv2.inRange(LAB_frame, lover_LAB, upper_LAB)

def print_sliders_values(event, x, y, flags, param):
    if event == cv2.EVENT_RBUTTONDOWN:
        print(f'lover_LAB = np.array([{sliders[0].value}, {sliders[2].value}, {sliders[4].value}])')
        print(f'upper_LAB = np.array([{sliders[1].value}, {sliders[3].value}, {sliders[5].value}])')
        print()

class Mask_slider:
    value = 0
    name = ''

    def __init__(self, Value, Name):
        self.value = Value
        self.name = Name

    def on_change(self, _value):
        self.value = _value
        update_binarise_image()
    
    def set_position(self, _value):
        cv2.setTrackbarPos(self.name, sliders_window, _value)
        self.value = _value

def change_errosion(value):
    global erosion
    erosion = value

def change_dilation(value):
    global dilation
    dilation = value

def change_errosion_it(value):
    global erosion_iter
    erosion_iter = value

def change_dilation_it(value):
    global dilation_iter
    dilation_iter = value


cv2.namedWindow(sliders_window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(sliders_window, 800, 200)
cv2.setMouseCallback(sliders_window, print_sliders_values)

for i in range(6):
    if i%2 == 0:
        slider = Mask_slider(lover_LAB[i//2], sliders_name[i])
        sliders.append(slider)
        cv2.createTrackbar(sliders_name[i], sliders_window, lover_LAB[i//2], 255, slider.on_change)
    else:
        slider = Mask_slider(upper_LAB[i//2], sliders_name[i])
        sliders.append(slider)
        cv2.createTrackbar(sliders_name[i], sliders_window, upper_LAB[i//2], 255, slider.on_change)

cv2.createTrackbar("Errosion", sliders_window, 0, 10, change_errosion)
cv2.createTrackbar("Errosion_iter", sliders_window, 0, 10, change_errosion_it)
cv2.createTrackbar("Dialation", sliders_window, 0, 10, change_dilation)
cv2.createTrackbar("Dialation_iter", sliders_window, 0, 10, change_dilation_it)

def print_pixel_value(event, x, y, flags, param):
    global pixels_min, pixels_max, sliders
    frame = LAB_frame[y][x]
    if event == cv2.EVENT_RBUTTONDOWN:
        pixels_min = [min(pixels_min[i], frame[i]) for i in range(3)]
        pixels_max = [max(pixels_max[i], frame[i]) for i in range(3)]
        print(f'Min {pixels_min}')
        print(f'Max {pixels_max}')
    if event == cv2.EVENT_LBUTTONDBLCLK:
        pixels_min = [255, 255, 255]
        pixels_max = [0, 0, 0]
    if event == cv2.EVENT_RBUTTONDBLCLK:
        for i in range(6):
            if i%2 == 0:
                sliders[i].set_position(pixels_min[i//2])
                
            else:
                sliders[i].set_position(pixels_max[i//2])
                

frame = cv2.imread("images/Olymp.png")
#frame_raw = cv2.imread("camera_calibration_images/IMG_20240620_033348.jpg")
frame = cv2.resize(frame, resolution, interpolation=cv2.INTER_LANCZOS4)
print(frame.shape)

transform_matrix = cv2.getPerspectiveTransform(corners_points, field_square)
transformed_frame = cv2.warpPerspective(frame, transform_matrix, (1400, 1400))
#transformed_frame = frame


cv2.imshow(window_name, frame)
cv2.moveWindow(window_name, window_origin[0], window_origin[1])
cv2.setMouseCallback(window_name, print_mouse_coordinates)

LAB_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
cv2.imshow(LAB_image_window, LAB_frame)



cv2.setMouseCallback(LAB_image_window, print_pixel_value)

update_binarise_image()
print(masked_image_window)



aruco_params = cv2.aruco.DetectorParameters()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

#corners, ids, rejected = aruco_detector.detectMarkers(frame_raw)

#print(ids)

#cv2.imshow("Raw", frame_raw)

while(True): 
    
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow(window_name, frame)

    final_frame = masked_frame
    if erosion > 0 and erosion_iter > 0:
        kernel = np.ones((erosion,erosion), np.uint8)
        final_frame = cv2.erode(final_frame, kernel, iterations=erosion_iter)
    if dilation > 0 and dilation_iter > 0:
        kernel = np.ones((dilation,dilation), np.uint8)
        final_frame = cv2.dilate(final_frame, kernel, iterations=dilation_iter)
    cv2.imshow(masked_image_window, final_frame)
    
    # cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q') or not(cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)):
        break

cv2.destroyAllWindows()