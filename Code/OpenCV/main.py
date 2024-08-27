import queue
from turtle import st
from unittest import skip
import cv2
import numpy as np
from itertools import permutations
import socket
import struct

resolution = (1280, 720)
window_origin = (0, 0)
window_name = "Field"

square_dimentions = 87.5
u_turn_weight = 1.8
turn_weight = 1

put_down_approach_distance = 0.6
destinations_avoidance_angle = 30

pipes = list()
accurate_destinations = list()
rounded_destinations = list()
rounded_destinations_cor = list()
field = [[0 for i in range(8)] for j in range(8)]
field_unmodified = [[0 for i in range(8)] for j in range(8)]
entrances = list()
entrances_cor = list()
blocked_squares_cor = list()

pick_up_points = list()
pick_up_points_dict = dict()
put_down_points = list()
start_point = tuple

start_point = (0, 0, 2)

robot_commands = list()

'''corners_points = np.float32([[300, 49], [930, 67],
                             [77, 692], [1125, 696]])'''
#corners_points = np.float32([[338, 33], [992, 32],
#                             [141, 701], [1241, 689]]) #Field1_angle1
#corners_points = np.float32([[300, 39], [945, 41],
#                             [44, 685], [1129, 706]]) #Field2_angle4
#corners_points = np.float32([[320, 37], [971, 32],
#                             [112, 701], [1210, 688]]) #Field1_angle2
#corners_points = np.float32([[286, 41], [936, 43],
#                             [33, 702], [1129, 707]]) #Field1_angle3

#corners_points = np.float32([[292, 31], [941, 45],
#                             [21, 685], [1118, 716]]) #Field1_angle4

#corners_points = np.float32([[363, 60], [997, 64],
#                             [176, 704], [1264, 696]]) #Field3_angle4
corners_points = np.float32([[370, 67], [945, 66],
                             [198, 673], [1108, 677]]) #Field3_angle4


field_square = np.float32([[-2, -5], [702, -5],
                           [-2, 702], [702, 702]])

field_square = np.float32([[0, 0], [700, 0],
                           [0, 700], [700, 700]])

extended_field_square = np.float32([[0, 45], [700, 45],
                           [0, 700], [700, 700]])

#stream = cv2.VideoCapture("rtsp://192.168.0.86:8554/live", cv2.CAP_ANY)
#ret, frame = stream.read()
#stream.release()
frame = cv2.imread("images/Olymp.png")
frame = cv2.resize(frame, resolution, interpolation=cv2.INTER_LINEAR)

transform_matrix = cv2.getPerspectiveTransform(corners_points, field_square)
transformed_frame = cv2.warpPerspective(frame, transform_matrix, (700, 700))

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result

transformed_frame = rotate_image(transformed_frame, 0)
#transformed_frame = cv2.flip(transformed_frame, 0)

LAB_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2LAB)


def detect_robot(frame):
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    corners, ids, rejected = aruco_detector.detectMarkers(frame)
    print(corners)
    x,y,w,h = cv2.boundingRect(corners[0])
    x_cor = x+w/2
    y_cor = y+h/2
    robots_cor = [round((x_cor/square_dimentions)+0.5)-1, round(((y_cor)/square_dimentions)+0.5)]

    marker_size = 30
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    fx = 700.0  # Focal length in pixels (x-axis)
    fy = 700.0  # Focal length in pixels (y-axis)
    cx = 350.0  # Principal point (x-coordinate)
    cy = 350.0  # Principal point (y-coordinate)
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((1, 5), dtype=np.float32)

    _, rvec, tvec = cv2.solvePnP(marker_points, corners[0], camera_matrix, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    aruco_angle = np.degrees(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))
    aruco_angle = (abs(aruco_angle - 180)+45)//90
    print("afasfsaf", aruco_angle)
    robot_angle = (aruco_angle+2)%4
    #robot_angle = aruco_angle
    robots_cor.append(robot_angle)

    return tuple(robots_cor)

def detect_destinations(debug = 0):
    lover_LAB = np.array([136, 88, 159])
    upper_LAB = np.array([187, 97, 164])

    lover_LAB = np.array([116, 80, 137])
    upper_LAB = np.array([165, 112, 160])#Field1_angle2
    y_shift = 35
    aprox_const = 0.065

    masked_frame = cv2.inRange(LAB_frame, lover_LAB, upper_LAB)

    final_frame = masked_frame
    kernel = np.ones((3,3), np.uint8)
    final_frame = cv2.erode(final_frame, kernel, iterations=4)
    kernel = np.ones((4,4), np.uint8)
    final_frame = cv2.dilate(final_frame, kernel, iterations=9)
    final_frame = cv2.rectangle(final_frame, (0, 0), (700, 700), 0, 1)

    cnts = cv2.findContours(final_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, aprox_const * peri, True)
        if len(approx) == 4:
            x,y,w,h = cv2.boundingRect(approx)
            x = x+w/2
            y = y+h/2
            if debug:
                print(round((x/square_dimentions+0.5)*2)/2)
                print(round((y/square_dimentions+0.5)*2)/2)
                print(w>h)
                print()
            if (round((x/square_dimentions)+0.5)-1, round(((y)/square_dimentions)+0.5)-1) != start_point[:-1]:
                accurate_destinations.append((round((x/square_dimentions)+0.5), round((y/square_dimentions+0.5)*2)/2, w>h))
                #blocked_squares_cor.append((round((x/square_dimentions)+0.5), round(((y)/square_dimentions)+0.5)))
                rounded_destinations.append((round((x/square_dimentions)+0.5), round(((y)/square_dimentions)+0.5), w>h))
                rounded_destinations_cor.append((round((x/square_dimentions)+0.5)-1, round(((y)/square_dimentions)+0.5)-1))
    rounded_destinations.sort()
    rounded_destinations_cor.sort()

def detect_layers(debug = 0):
    white_lover_LAB = np.array([8, 126, 127])
    white_upper_LAB = np.array([225, 131, 133])
    white_lover_LAB = np.array([8, 123, 114])
    white_upper_LAB = np.array([240, 133, 143])#Field1_angle2

    white_dilation = 2
    white_diletion_iter = 2

    brown_lover_LAB = np.array([19, 131, 143])
    brown_upper_LAB = np.array([187, 143, 158])
    brown_lover_LAB = np.array([19, 127, 136])
    brown_upper_LAB = np.array([206, 144, 160])#Field1_angle2

    brown_errosion = 4
    brown_errosion_iter = 4

    white_mask = cv2.inRange(LAB_frame, white_lover_LAB, white_upper_LAB)
    kernel = np.ones((white_dilation,white_dilation), np.uint8)
    white_mask = cv2.dilate(white_mask, kernel, iterations=white_diletion_iter)

    brown_mask = cv2.inRange(LAB_frame, brown_lover_LAB, brown_upper_LAB)
    kernel = np.ones((brown_errosion,brown_errosion), np.uint8)
    brown_mask = cv2.erode(brown_mask, kernel, iterations=brown_errosion_iter)

    if debug:
        cv2.imshow("White", white_mask)
        cv2.imshow("Brown", brown_mask)

    for i in range(8):
        for j in range(8):
            x = i*square_dimentions
            y = j*square_dimentions
            white_sum = 0
            brown_sum = 0
            for k in range(int(square_dimentions)-1):
                for q in range(int(square_dimentions)-1):
                    yy = round(y+k)
                    xx = round(x+q)
                    if yy >= 699:
                        yy = 699

                    if xx >= 699:
                        xx = 699

                    white_sum += int(white_mask[xx][yy]==255)
                    brown_sum += int(brown_mask[xx][yy]==255)
            if debug:
                print(brown_sum>white_sum, brown_sum, white_sum, i, j)
            field[i][j] = brown_sum>white_sum
            field_unmodified[i][j] = brown_sum>white_sum
        if debug:
            print()

def detect_pipes(debug = 0):
    lover_LAB = np.array([71, 119, 124])
    upper_LAB = np.array([152, 127, 136])

    lover_LAB = np.array([74, 119, 112])
    upper_LAB = np.array([159, 134, 124])

    y_shift = 45

    extended_transform_matrix = cv2.getPerspectiveTransform(corners_points, extended_field_square)
    pipes_frame = cv2.warpPerspective(frame, extended_transform_matrix, (700, 700))
    pipes_frame = cv2.cvtColor(pipes_frame, cv2.COLOR_BGR2LAB)

    masked_frame = cv2.inRange(pipes_frame, lover_LAB, upper_LAB)
    final_frame = masked_frame

    kernel = np.ones((2,2), np.uint8)
    final_frame = cv2.erode(final_frame, kernel, iterations=3)
    kernel = np.ones((3,3), np.uint8)
    final_frame = cv2.dilate(final_frame, kernel, iterations=3)
    final_frame = cv2.rectangle(final_frame, (0, 0), (700, 700), 0, 1)

    for i in range(8):
        cv2.line(final_frame, (0, int(i*square_dimentions+0.5)), (700, int(i*square_dimentions+0.5)), 0, 1)
    
    cv2.imshow("Tubes", final_frame)


    cnts = cv2.findContours(final_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        x,y,w,h = cv2.boundingRect(approx)
        if w*h > 250 and w > 10 and h > 10:
            x = x+w/2
            y = y+h/2
            if debug:
                print(round((x/square_dimentions)+0.5))
                print(round(((y+y_shift)/square_dimentions)+0.5))
                print(w>h)
                print()
            if (round((x/square_dimentions)+0.5)-1, round(((y)/square_dimentions)+0.5)-1) != start_point[:-1]:
                pipes.append((round((x/square_dimentions)+0.5), round(((y)/square_dimentions)+0.5), int(w>h)))
                blocked_squares_cor.append((round((x/square_dimentions)+0.5), round(((y)/square_dimentions)+0.5)))
    
    delete_pipes_ind = 0
    if len(pipes) == 4:
        for i in range(len(pipes)):
            for j in range(len(pipes)):
                if pipes[i][0] == pipes[j][0] and pipes[i][1] + 1 == pipes[j][1]:
                    delete_pipes_ind = i
        pipes.pop(delete_pipes_ind)
        blocked_squares_cor.pop(delete_pipes_ind)

def detect_entrances(debug = 0):
    red_lover_LAB = np.array([90, 146, 152])
    red_upper_LAB = np.array([123, 183, 166])

    red_lover_LAB = np.array([73, 150, 129])
    red_upper_LAB = np.array([140, 186, 156])

    red_erode = 3
    red_erode_it = 4
    red_dilate = 4
    red_dilate_it = 6

    blue_lover_LAB = np.array([131, 108, 100])
    blue_upper_LAB = np.array([153, 122, 125])

    blue_lover_LAB = np.array([40, 126, 72])
    blue_upper_LAB = np.array([154, 159, 113])

    blue_erode = 2
    blue_erode_it = 2
    blue_dilate = 4
    blue_dilate_it = 4

    circular_const = 0.2

    red_mask = cv2.inRange(LAB_frame, red_lover_LAB, red_upper_LAB)

    kernel = np.ones((red_erode,red_erode), np.uint8)
    red_mask = cv2.erode(red_mask, kernel, iterations=red_erode_it)
    kernel = np.ones((red_dilate,red_dilate), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel, iterations=red_dilate_it)
    red_mask = cv2.rectangle(red_mask, (0, 0), (700, 700), 0, 2)
    red_mask = 255-red_mask
    cv2.imshow("Red", red_mask)

    blue_mask = cv2.inRange(LAB_frame, blue_lover_LAB, blue_upper_LAB)

    kernel = np.ones((blue_erode,blue_erode), np.uint8)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=blue_erode_it)
    kernel = np.ones((blue_dilate,blue_dilate), np.uint8)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=blue_dilate_it)
    blue_mask = cv2.rectangle(blue_mask, (0, 0), (700, 700), 0, 2)
    blue_mask = 255-blue_mask
    cv2.imshow("Blue", blue_mask)

    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 40
    params.maxThreshold = 255
    params.filterByArea = False
    params.filterByCircularity = True
    params.minCircularity = circular_const
    params.filterByConvexity = False
    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector_create(params)

    red_blobs = detector.detect(red_mask)
    blue_blobs = detector.detect(blue_mask)

    red_blobs_rounded = list()
    blue_blobs_rounded = list()
    for i in red_blobs:
        print((round(i.pt[0]/square_dimentions+0.5), round(i.pt[1]/square_dimentions+0.5)), blocked_squares_cor)
        if (round(i.pt[0]/square_dimentions+0.5)-1, round(i.pt[1]/square_dimentions+0.5)-1) != start_point[:-1] and (round(i.pt[0]/square_dimentions+0.5), round(i.pt[1]/square_dimentions+0.5)) not in blocked_squares_cor:
            red_blobs_rounded.append((round(i.pt[0]/square_dimentions+0.5), round(i.pt[1]/square_dimentions+0.5), i.pt[0], i.pt[1]))

    for i in blue_blobs:
        if (round(i.pt[0]/square_dimentions+0.5)-1, round(i.pt[1]/square_dimentions+0.5)-1) != start_point[:-1]:
            blue_blobs_rounded.append((round(i.pt[0]/square_dimentions+0.5), round(i.pt[1]/square_dimentions+0.5), i.pt[0], i.pt[1]))
    
    red_blobs_rounded.sort()
    blue_blobs_rounded.sort()
    print(red_blobs_rounded)
    print(blue_blobs_rounded)

    #cv2.waitKey(0)
    
    for i in range(min(len(red_blobs_rounded), len(blue_blobs_rounded))):
        entrance = list()
        entrance.append(red_blobs_rounded[i][0])
        entrance.append(red_blobs_rounded[i][1])
        if (blue_blobs_rounded[i][3] - red_blobs_rounded[i][3]) < -20 and abs(blue_blobs_rounded[i][2] - red_blobs_rounded[i][2]) < 20:
            entrance.append(0)
        elif (blue_blobs_rounded[i][2] - red_blobs_rounded[i][2]) < -20  and abs(blue_blobs_rounded[i][3] - red_blobs_rounded[i][3]) < 20:
            entrance.append(1) 
        elif (blue_blobs_rounded[i][3] - red_blobs_rounded[i][3]) > 20 and abs(blue_blobs_rounded[i][2] - red_blobs_rounded[i][2]) < 20:
            entrance.append(2)
        elif (blue_blobs_rounded[i][2] - red_blobs_rounded[i][2]) > 20 and abs(blue_blobs_rounded[i][3] - red_blobs_rounded[i][3]) < 20:
            entrance.append(3)
        else:
            entrance.append(0)
        entrances.append(tuple(entrance))
        blocked_squares_cor.append(tuple(entrance[:-1]))
        entrances_cor.append(tuple(entrance[:-1]))

def draw_elements():
    for i in pipes:
        if i[2]:
            cv2.rectangle(transformed_frame, (int((i[0] - 0.5)*square_dimentions - 25), int((i[1] - 0.5)*square_dimentions - 10)), (int((i[0] - 0.5)*square_dimentions + 25), int((i[1] - 0.5)*square_dimentions + 10)), (0, 255, 0), 2)
        else:
            cv2.rectangle(transformed_frame, (int((i[0] - 0.5)*square_dimentions - 10), int((i[1] - 0.5)*square_dimentions - 25)), (int((i[0] - 0.5)*square_dimentions + 10), int((i[1] - 0.5)*square_dimentions + 25)), (0, 255, 0), 2)   

    for i in accurate_destinations:
        if i[2]:
            cv2.rectangle(transformed_frame, (int((i[0] - 0.5)*square_dimentions - 25), int((i[1] - 0.5)*square_dimentions - 10)), (int((i[0] - 0.5)*square_dimentions + 25), int((i[1] - 0.5)*square_dimentions + 10)), (0, 255, 255), 2)
        else:
            cv2.rectangle(transformed_frame, (int((i[0] - 0.5)*square_dimentions - 10), int((i[1] - 0.5)*square_dimentions - 25)), (int((i[0] - 0.5)*square_dimentions + 10), int((i[1] - 0.5)*square_dimentions + 25)), (0, 255, 255), 2)

    for i in range(len(field)):
        for j in range(len(field[i])):
            if field[i][j]:
                cv2.circle(transformed_frame,(int((j+0.5)*square_dimentions), int((i+0.5)*square_dimentions)) , 5, (255, 0, 255), 2)
    
    for i in entrances:
        if i[2] == 0 or i[2] == 2:
            cv2.rectangle(transformed_frame, (int((i[0] - 0.5)*square_dimentions - 15), int((i[1] - 0.5)*square_dimentions - 5)), (int((i[0] - 0.5)*square_dimentions + 15), int((i[1] - 0.5)*square_dimentions + 5)), (255, 0, 0), 5)
        else:
            cv2.rectangle(transformed_frame, (int((i[0] - 0.5)*square_dimentions - 5), int((i[1] - 0.5)*square_dimentions - 15)), (int((i[0] - 0.5)*square_dimentions + 5), int((i[1] - 0.5)*square_dimentions + 15)), (255, 0, 0), 5)

start_point = detect_robot(LAB_frame)

detect_pipes()
detect_destinations()
detect_layers()
detect_entrances()

field[start_point[1]][start_point[0]] = 0

'''entrances.append((7, 5, 1))
entrances.append((7, 6, 3))
entrances_cor.append((7, 5))
entrances_cor.append((7, 6))
blocked_squares_cor.append((7, 5))
blocked_squares_cor.append((7, 6))

blocked_squares_cor.append((6, 2))
blocked_squares_cor.append((7, 7))
pipes.append((6, 2, 0))
pipes.append((7, 7, 1))

blocked_squares_cor.append((2, 1))
pipes.append((2, 1, 0))'''
#field[1][4] = 1
#entrances.append((5, 2, 2))
#entrances_cor.append((5, 2))
#blocked_squares_cor.append((5, 2))

draw_elements()

#cv2.waitKey(0)

graph_img = np.zeros((500,500,3), np.uint8)

graph = dict()
second_floor_squares = list()
leftover_nodes = [(i, j) for j in range(8) for i in range(8)]

def pregenerate_graph(node):
    global graph, leftover_nodes

    for shift in ((1, 0), (0, 1), (-1, 0), (0, -1)):
        if 8 > node[0]+shift[0] >= 0 and 8 > node[1]+shift[1] >= 0:
            if field[node[1]+shift[1]][node[0]+shift[0]] == field[node[1]][node[0]] and ((node[0]+shift[0]+1, node[1]+shift[1]+1) not in blocked_squares_cor) and ((node[0]+1, node[1]+1) not in blocked_squares_cor):
                if node in graph.keys():
                    graph[node].append((node[0]+shift[0], node[1]+shift[1], 1)) 
                else:
                    graph[node] = [(node[0]+shift[0], node[1]+shift[1], 1)]

    
    leftover_nodes.remove(node)

def geberate_graph():
    global graph, second_floor_squares, leftover_nodes, pick_up_points, pick_up_points_dict, put_down_points, field_unmodified

    for i in entrances_cor:
        field[i[1] -1][i[0] - 1] = 0
        field_unmodified[i[1] -1][i[0] - 1] = 1

    while (len(leftover_nodes) > 0):
        pregenerate_graph(leftover_nodes[0])

    for i in pipes:
        pick_points_unrestricted = list()
        pick_points = list()
        if i[2] == 0:
            pick_points_unrestricted.append((i[0]-2, i[1]-1, 2))
            pick_points_unrestricted.append((i[0], i[1]-1, 0))
        elif i[2] == 1:
            pick_points_unrestricted.append((i[0]-1, i[1]-2, 1))
            pick_points_unrestricted.append((i[0]-1, i[1], 3))
        counter = 0
        for j in pick_points_unrestricted:
            if 8 > j[0] >= 0 and 8 > j[1] >= 0 and ((j[0] + 1, j[1] + 1) not in blocked_squares_cor) and field[j[1]][j[0]] == field[i[1]-1][i[0]-1]:
                pick_points.append((j[0], j[1], j[2], 0))
                pick_up_points_dict[(j[0], j[1])] = (j[2], 0)
            if 8 > j[0] >= 0 and 8 > j[1] >= 0 and (j[0] + 1, j[1] + 1) in entrances_cor:
                entrance = entrances[entrances_cor.index((j[0] + 1, j[1] + 1))]

                if entrance[2]%2 == i[2] and abs(i[2] - entrance[2]//2) == (1-counter) and field[i[1] - 1][i[0] - 1] == 1:
                    pick_points.append((j[0], j[1], j[2], 1))
                    pick_up_points_dict[(j[0], j[1])] = (j[2], 1)
                if entrance[2]%2 == i[2] and abs(i[2] - entrance[2]//2) == counter and field[i[1] - 1][i[0] - 1] == 0:
                    pick_points.append((j[0], j[1], j[2], 2))
                    pick_up_points_dict[(j[0], j[1])] = (j[2], 2)
            counter += 1
        pick_up_points.append(pick_points)

    for i in entrances:
        if i[2] == 0:
            lover = (i[0], i[1] - 1)
            upper = (i[0] - 2, i[1] - 1)
        elif i[2] == 1:
            lover = (i[0] - 1, i[1] - 2)
            upper = (i[0] - 1, i[1])
        elif i[2] == 2:
            lover = (i[0] - 2, i[1] - 1)
            upper = (i[0], i[1] - 1)
        elif i[2] == 3:
            lover = (i[0] - 1, i[1])
            upper = (i[0] - 1, i[1] - 2)

        if 8 > lover[0] >= 0 and 8 > lover[1] >=0 and field[lover[1]][lover[0]] == 0 and ((lover[0]+1, lover[1]+1) not in [tuple(j[:-1]) for j in pipes]):
            if (lover[0], lover[1]) in graph.keys():
                graph[(lover[0], lover[1])].append((i[0] - 1, i[1] - 1, 1.5))
            else:
                graph[(lover[0], lover[1])] = [(i[0] - 1, i[1] - 1, 1.5)]
            graph[(i[0] - 1, i[1] - 1)] = [(lover[0], lover[1], 1.5)]
        if 8 > upper[0] >= 0 and 8 > upper[1] >=0 and field[upper[1]][upper[0]] == 1 and ((upper[0]+1, upper[1]+1) not in [tuple(j[:-1]) for j in pipes]):
            if (upper[0], upper[1]) in graph.keys():
                graph[(upper[0], upper[1])].append((i[0] - 1, i[1] - 1, 1.5))
            else:
                graph[(upper[0], upper[1])] = [(i[0] - 1, i[1] - 1, 1.5)]
            if (i[0] - 1, i[1] - 1) in graph.keys():
                graph[(i[0] - 1, i[1] - 1)].append((upper[0], upper[1], 1.5))
            else:
                graph[(i[0] - 1, i[1] - 1)] = [(upper[0], upper[1], 1.5)]
        if 8 > upper[0] >= 0 and 8 > upper[1] >=0 and (upper[0]+1, upper[1]+1) in entrances_cor and abs(i[2] - entrances[entrances_cor.index((upper[0]+1, upper[1]+1))][2]) == 2:
            if (upper[0], upper[1]) in graph.keys():
                graph[(upper[0], upper[1])].append((i[0] - 1, i[1] - 1, 3))
            else:
                graph[(upper[0], upper[1])] = [(i[0] - 1, i[1] - 1, 3)]
            if (i[0] - 1, i[1] - 1) in graph.keys():
                graph[(i[0] - 1, i[1] - 1)].append((upper[0], upper[1], 3))
            else:
                graph[(i[0] - 1, i[1] - 1)] = [(upper[0], upper[1], 3)]
        
    for i in rounded_destinations:
        put_points_unrestricted = list()
        put_points = list()
        if i[2] == 0:
            put_points_unrestricted.append((i[0]-2, i[1]-1, 2))
            put_points_unrestricted.append((i[0], i[1]-1, 0))
        elif i[2] == 1:
            put_points_unrestricted.append((i[0]-1, i[1]-2, 1))
            put_points_unrestricted.append((i[0]-1, i[1], 3))
        
        for j in put_points_unrestricted:
            if 8 > j[0] >= 0 and 8 > j[1] >= 0 and ((j[0] + 1, j[1] + 1) not in blocked_squares_cor) and field[j[1]][j[0]] == field[i[1]-1][i[0]-1]:
                put_points.append(j)
        put_down_points.append(put_points[0])

    for i in graph.keys():
        if (i[0], i[1]) in rounded_destinations_cor:
            target_point = tuple()     
            for j in graph[i]:
                if j[:-1] in [k[:-1] for k in put_down_points]:
                    target_point = j
                    break
            graph[i].remove(target_point)
            if graph[i] == list():
                del graph[i]
        
        if i in [q[:-1] for q in put_down_points]:
            target_point = tuple()     
            for j in graph[i]:
                if (j[0], j[1]) in rounded_destinations_cor:
                    target_point = j
                    break
            graph[i].remove(target_point)
            if graph[i] == list():
                del graph[i]
    
    for i in range(len(graph[rounded_destinations_cor[1]])):
        graph[rounded_destinations_cor[1]][i] = (graph[rounded_destinations_cor[1]][i][0], graph[rounded_destinations_cor[1]][i][1], 1.1)
    
    point_ind = [i[:-1] for i in graph[rounded_destinations_cor[0]]].index(rounded_destinations_cor[1])
    graph[rounded_destinations_cor[0]][point_ind] = (graph[rounded_destinations_cor[0]][point_ind][0], graph[rounded_destinations_cor[0]][point_ind][1], 1.1)

    point_ind = [i[:-1] for i in graph[rounded_destinations_cor[2]]].index(rounded_destinations_cor[1])
    graph[rounded_destinations_cor[2]][point_ind] = (graph[rounded_destinations_cor[2]][point_ind][0], graph[rounded_destinations_cor[2]][point_ind][1], 1.1)

    for i in range(len(field)):
        for j in range(len(field[i])):
            if field[i][j] == 1 and ((j+1, i+1) not in entrances_cor):
                second_floor_squares.append((j, i)) 

geberate_graph()

def draw_graph():
    for i in graph.keys():
        if i in second_floor_squares:
            cv2.circle(graph_img, (int((i[0]+0.5)*50), int((i[1]+0.5)*50)), 3, (255, 0, 255), 10)
        else:
            cv2.circle(graph_img, (int((i[0]+0.5)*50), int((i[1]+0.5)*50)), 3, (150, 150, 255), 10)
        for j in graph[i]:
            if j[2] == 1 or j[2] == 1.1:
                cv2.line(graph_img, (int((i[0]+0.5)*50), int((i[1]+0.5)*50)), (int((j[0]+0.5)*50), int((j[1]+0.5)*50)), (0, 255, 255), 2)
            if j[2] == 1.5:
                cv2.line(graph_img, (int((i[0]+0.5)*50), int((i[1]+0.5)*50)), (int((j[0]+0.5)*50), int((j[1]+0.5)*50)), (255, 0, 0), 2)
            if j[2] == 3:
                cv2.line(graph_img, (int((i[0]+0.5)*50), int((i[1]+0.5)*50)), (int((j[0]+0.5)*50), int((j[1]+0.5)*50)), (255, 255, 0), 2)
    for i in pick_up_points:
        for j in i:
            if j[3] == 0:
                cv2.circle(graph_img, (int((j[0]+0.5)*50), int((j[1]+0.5)*50)), 3, (0, 0, 255), 10)
            elif j[3] == 1:
                cv2.circle(graph_img, (int((j[0]+0.5)*50), int((j[1]+0.5)*50)), 3, (0, 100, 255), 10)
            elif j[3] == 2:
                cv2.circle(graph_img, (int((j[0]+0.5)*50), int((j[1]+0.5)*50)), 3, (0, 200, 255), 10)
    for i in put_down_points:
        cv2.circle(graph_img, (int((i[0]+0.5)*50), int((i[1]+0.5)*50)), 3, (0, 255, 0), 10)
draw_graph()
cv2.imshow("Graph", graph_img)
cv2.imshow(window_name, transformed_frame)

print()
print(pipes)
print(pick_up_points, pick_up_points_dict)
print(rounded_destinations_cor)
print(put_down_points)
print(entrances)

#cv2.waitKey(0)

def generate_rotation_compensation(rotation_):
    return [(rotation_ % 2) * turn_weight + (rotation_ == 2) * u_turn_weight,
            (1 - (rotation_ % 2)) * turn_weight + (rotation_ == 3) * u_turn_weight,
            (rotation_ % 2) * turn_weight + (rotation_ == 0) * u_turn_weight,
            (1 - (rotation_ % 2)) * turn_weight + (rotation_ == 1) * u_turn_weight]

def dijkstra (Graph, start_point):
    distances = dict()
    for i in Graph.keys():
        distances[i] = [999999, 999999, 999999, 999999]
    
    rotation = (start_point[-1])%4
    distances[start_point[:-1]] = generate_rotation_compensation(rotation)

    start_point = start_point[:-1]

    points_queue = list()
    visited = list()

    points_queue.append(start_point)

    while len(points_queue) > 0:
        current_point = points_queue.pop(0)
        if current_point in visited:
            continue
        visited.append(current_point)

        for neighbor_point in graph[current_point]:
            neighbor_distances = [9999999]*4
            neighbor_direction = 0
            if neighbor_point[1] == current_point[1] and neighbor_point[0] < current_point[0]:
                neighbor_distances[0] = distances[current_point][0] + 0 + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + turn_weight + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + u_turn_weight + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + turn_weight + neighbor_point[2]
                neighbor_direction = 0
            if neighbor_point[0] == current_point[0] and neighbor_point[1] > current_point[1]:
                neighbor_distances[0] = distances[current_point][0] + turn_weight + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + 0 + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + turn_weight + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + u_turn_weight + neighbor_point[2]
                neighbor_direction = 1
            if neighbor_point[1] == current_point[1] and neighbor_point[0] > current_point[0]:
                neighbor_distances[0] = distances[current_point][0] + u_turn_weight + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + turn_weight + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + 0 + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + turn_weight + neighbor_point[2]
                neighbor_direction = 2
            if neighbor_point[0] == current_point[0] and neighbor_point[1] < current_point[1]:
                neighbor_distances[0] = distances[current_point][0] + turn_weight + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + u_turn_weight + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + turn_weight + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + 0 + neighbor_point[2]
                neighbor_direction = 3
            flag = 0
            for i in range(4):
                if neighbor_distances[i] < distances[neighbor_point[:-1]][neighbor_direction]:
                    distances[neighbor_point[:-1]][neighbor_direction] = neighbor_distances[i]
                    flag = 1
            if flag:
                points_queue.append(neighbor_point[:-1])

    return distances

def dijkstra_trajectory (Graph, start_point, end_point):
    start_point_full = start_point
    end_point_full = end_point

    distances = dict()
    came_from = dict()
    for i in Graph.keys():
        distances[i] = [999999, 999999, 999999, 999999]
        came_from[i] = [(), (), (), ()]
    
    rotation = (start_point[-1])%4
    distances[start_point[:-1]] = generate_rotation_compensation(rotation)

    start_point = start_point[:-1]
    end_point = end_point[:-1]

    points_queue = list()
    visited = list()

    points_queue.append(start_point)

    while len(points_queue) > 0:
        current_point = points_queue.pop(0)
        if current_point in visited:
            continue
        visited.append(current_point)

        for neighbor_point in graph[current_point]:
            neighbor_distances = [9999999]*4
            neighbor_direction = 0
            if neighbor_point[1] == current_point[1] and neighbor_point[0] < current_point[0]:
                neighbor_distances[0] = distances[current_point][0] + 0 + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + turn_weight + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + u_turn_weight + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + turn_weight + neighbor_point[2]
                neighbor_direction = 0
            if neighbor_point[0] == current_point[0] and neighbor_point[1] > current_point[1]:
                neighbor_distances[0] = distances[current_point][0] + turn_weight + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + 0 + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + turn_weight + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + u_turn_weight + neighbor_point[2]
                neighbor_direction = 1
            if neighbor_point[1] == current_point[1] and neighbor_point[0] > current_point[0]:
                neighbor_distances[0] = distances[current_point][0] + u_turn_weight + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + turn_weight + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + 0 + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + turn_weight + neighbor_point[2]
                neighbor_direction = 2
            if neighbor_point[0] == current_point[0] and neighbor_point[1] < current_point[1]:
                neighbor_distances[0] = distances[current_point][0] + turn_weight + neighbor_point[2]
                neighbor_distances[1] = distances[current_point][1] + u_turn_weight + neighbor_point[2]
                neighbor_distances[2] = distances[current_point][2] + turn_weight + neighbor_point[2]
                neighbor_distances[3] = distances[current_point][3] + 0 + neighbor_point[2]
                neighbor_direction = 3
            flag = 0
            for i in range(4):
                if neighbor_distances[i] < distances[neighbor_point[:-1]][neighbor_direction]:
                    distances[neighbor_point[:-1]][neighbor_direction] = neighbor_distances[i]
                    came_from[neighbor_point[:-1]][neighbor_direction] = current_point
                    flag = 1
            if flag:
                points_queue.append(neighbor_point[:-1])
    
    if start_point == end_point:
        return [list(start_point_full) + [0], list(end_point_full) + [0]]

    trajectory = list()
    current_point = end_point
    rotation = (end_point_full[-1])%4
    rotation_add = generate_rotation_compensation(rotation)
    distances[end_point] = [distances[end_point][i] + rotation_add[i] for i in range(len(rotation_add))]
    came_from_ind = np.argmin(distances[current_point])
    previous_point = current_point
    distancess = list()
    angles = list()
    flag = 0
    while current_point != start_point:
        trajectory.append([came_from[current_point][came_from_ind][0], came_from[current_point][came_from_ind][1], 0, 0])
        if flag:
            distancess.append(graph[previous_point][[i[:-1] for i in graph[previous_point]].index(current_point)][-1])
        flag = 1
        angles.append(came_from_ind)

        previous_point = current_point
        current_point = came_from[current_point][came_from_ind]
        rotation = came_from_ind
        rotation_add = generate_rotation_compensation(rotation)
        came_from_ind = np.argmin([distances[current_point][i] + rotation_add[i] for i in range(4)])
    
    distancess.append(graph[current_point][[i[:-1] for i in graph[current_point]].index(previous_point)][-1])
    angles.append(came_from_ind)

    angles.reverse()
    distancess.append(0)
    distancess.reverse()
    trajectory = list(reversed(trajectory[:-1]))
    trajectory = [list(start_point_full) + [0]] + trajectory + [list(end_point_full) + [0]]
    if angles[-1] != trajectory[-1][2]:
        trajectory.append(list(end_point_full) + [0])
        distancess.append(0)
        angles.append(end_point_full[-1])
    for i in range(1, len(trajectory)):
        trajectory[i][3] = distancess[i]
        trajectory[i][2] = angles[i]
    return trajectory


def calculate_min_distance(distances, rotation):
    rotation = (rotation)%4
    rotation_comp = [(rotation % 2) * turn_weight + (rotation == 2) * u_turn_weight,
                    (1 - (rotation % 2)) * turn_weight + (rotation == 3) * u_turn_weight,
                    (rotation % 2) * turn_weight + (rotation == 0) * u_turn_weight,
                    (1 - (rotation % 2)) * turn_weight + (rotation == 1) * u_turn_weight]
    
    return min([distances[i] + rotation_comp[i] for i in range(len(distances))])

min_distance = 9999999
min_journey = list()
pipe_pick = [0]*3

for pipes_permutations in permutations((0, 1, 2)):
    for pipe_pick[0] in range(len(pick_up_points[0])):
        for pipe_pick[1] in range(len(pick_up_points[1])):
            for pipe_pick[2] in range(len(pick_up_points[2])):
                for put_points_permutations in permutations((0, 1, 2)):
                    tmp_distance = calculate_min_distance(dijkstra(graph, start_point)[pick_up_points[pipes_permutations[0]][pipe_pick[pipes_permutations[0]]][:-2]], pick_up_points[pipes_permutations[0]][pipe_pick[pipes_permutations[0]]][-2])
                    tmp_distance += calculate_min_distance(dijkstra(graph, pick_up_points[pipes_permutations[0]][pipe_pick[pipes_permutations[0]]][:-1])[pick_up_points[pipes_permutations[1]][pipe_pick[pipes_permutations[1]]][:-2]], pick_up_points[pipes_permutations[1]][pipe_pick[pipes_permutations[1]]][-2])
                    tmp_distance += calculate_min_distance(dijkstra(graph, pick_up_points[pipes_permutations[1]][pipe_pick[pipes_permutations[1]]][:-1])[put_down_points[put_points_permutations[0]][:-1]], put_down_points[put_points_permutations[0]][-1])
                    tmp_distance += calculate_min_distance(dijkstra(graph, put_down_points[put_points_permutations[0]])[put_down_points[put_points_permutations[1]][:-1]], put_down_points[put_points_permutations[1]][-1])
                    tmp_distance += calculate_min_distance(dijkstra(graph, put_down_points[put_points_permutations[1]])[pick_up_points[pipes_permutations[2]][pipe_pick[pipes_permutations[2]]][:-2]], pick_up_points[pipes_permutations[2]][pipe_pick[pipes_permutations[2]]][-2])
                    tmp_distance += calculate_min_distance(dijkstra(graph, pick_up_points[pipes_permutations[2]][pipe_pick[pipes_permutations[2]]][:-1])[put_down_points[put_points_permutations[1]][:-1]], put_down_points[put_points_permutations[1]][-1])
                    if tmp_distance < min_distance:
                        min_distance = tmp_distance
                        min_journey = (pipes_permutations, put_points_permutations, pipe_pick)

journey_points_full = list()
journey_points_full.append(dijkstra_trajectory(graph, start_point, pick_up_points[min_journey[0][0]][min_journey[2][0]][:-1]))
journey_points_full.append(dijkstra_trajectory(graph, pick_up_points[min_journey[0][0]][min_journey[2][0]][:-1], pick_up_points[min_journey[0][1]][min_journey[2][1]][:-1]))
journey_points_full.append(dijkstra_trajectory(graph, pick_up_points[min_journey[0][1]][min_journey[2][1]][:-1], put_down_points[min_journey[1][0]]))
journey_points_full.append(dijkstra_trajectory(graph, put_down_points[min_journey[1][0]], put_down_points[min_journey[1][1]]))
journey_points_full.append(dijkstra_trajectory(graph, put_down_points[min_journey[1][1]], pick_up_points[min_journey[0][2]][min_journey[2][2]][:-1]))
journey_points_full.append(dijkstra_trajectory(graph, pick_up_points[min_journey[0][2]][min_journey[2][2]][:-1], put_down_points[min_journey[1][2]]))
'''comand_type:
1 - move_forward_till_line_on_line
2 - move_forward_distance_on_line
3 - move_forward_free
4 - move_forward_till_line_right_shift
5 - move_forward_till_line_left_shift
6 - rotate
7 - grab_pipe_same_high
8 - grab_pipe_above
9 - grab_pipe_below
10 - put_pipe'''

print(*journey_points_full, sep = "\n")
#print(rounded_destinations_cor)

#print(entrances_cor)

for (ind, journey_leg) in enumerate(journey_points_full):
    for i in range(1, len(journey_leg)): 
        if journey_leg[i][-2] != journey_leg[i-1][-2]:
            robot_commands.append((6, 90*((journey_leg[i][-2] - journey_leg[i-1][-2] + 6)%4 - 2)))
        if (journey_leg[i][0]+1, journey_leg[i][1]+1) in entrances_cor or (journey_leg[i-1][0]+1, journey_leg[i-1][1]+1) in entrances_cor:
            if (journey_leg[i][0]+1, journey_leg[i][1]+1) in entrances_cor and (journey_leg[i-1][0]+1, journey_leg[i-1][1]+1) not in entrances_cor and field[journey_leg[i-1][1]][journey_leg[i-1][0]] == 0:
                robot_commands.append((2, 315))
            elif (journey_leg[i][0]+1, journey_leg[i][1]+1) not in entrances_cor and (journey_leg[i-1][0]+1, journey_leg[i-1][1]+1) in entrances_cor and field[journey_leg[i][1]][journey_leg[i][0]] == 1:
                robot_commands.append((2, 320))
            elif (journey_leg[i][0]+1, journey_leg[i][1]+1) in entrances_cor and (journey_leg[i-1][0]+1, journey_leg[i-1][1]+1) not in entrances_cor and field[journey_leg[i-1][1]][journey_leg[i-1][0]] == 1:
                robot_commands.append((2, 260))
            elif (journey_leg[i][0]+1, journey_leg[i][1]+1) not in entrances_cor and (journey_leg[i-1][0]+1, journey_leg[i-1][1]+1) in entrances_cor and field[journey_leg[i][1]][journey_leg[i][0]] == 0:
                robot_commands.append((2, 295))
            elif (journey_leg[i][0]+1, journey_leg[i][1]+1) in entrances_cor and (journey_leg[i-1][0]+1, journey_leg[i-1][1]+1) in entrances_cor:
                robot_commands.append((2, 270))
        elif journey_leg[i][-1] == 1 and tuple(journey_leg[i][:2]) in rounded_destinations_cor and tuple(journey_leg[i-1][:2]) not in rounded_destinations_cor:
            avoidance_direction = ((journey_leg[i][-2]//2)*2-1)
            robot_commands.append((6, destinations_avoidance_angle*avoidance_direction))
            robot_commands.append((3, 1/np.sin(np.radians(destinations_avoidance_angle))))
            robot_commands.append((6, -1*destinations_avoidance_angle*avoidance_direction))
        elif journey_leg[i][-1] == 1 and tuple(journey_leg[i-1][:2]) in rounded_destinations_cor and tuple(journey_leg[i][:2]) not in rounded_destinations_cor:
            avoidance_direction = ((journey_leg[i][-2]//2)*2-1)
            robot_commands.append((6, -1*destinations_avoidance_angle*avoidance_direction))
            robot_commands.append((3, 1/np.sin(np.radians(destinations_avoidance_angle))))
            robot_commands.append((6, destinations_avoidance_angle*avoidance_direction))
        elif journey_leg[i][-1] == 1.1 and tuple(journey_leg[i][:2]) in rounded_destinations_cor:
            robot_commands.append((4 + journey_leg[i][-2]//2, 1))
        else:
            if field[journey_leg[i][1]][journey_leg[i][0]] == 1:
                robot_commands.append((2, 300))
            else:
                robot_commands.append((1, 1))
    if ind in (0, 1, 4):
        current_pick_up_point_type = pick_up_points_dict[tuple(journey_leg[-1][:-2])][-1]
        robot_commands.append((9+current_pick_up_point_type, 0))
    if ind in (2, 3, 5):
        robot_commands.append((12, 0))

robot_commands_optimised = list()
cumulitive_dist = 0
command_type = 0

skip_flag = 0
for command in robot_commands:
    if skip_flag and command[0] == 2:
        skip_flag = 0
        continue

    if command[0] in (1, 3, 4, 5):
        command_type = command[0]
    
    if command[0] in(8, 9):
        skip_flag = 1

    if command[0] == command_type:
        cumulitive_dist += command[1]
    else:
        if cumulitive_dist != 0:
            robot_commands_optimised.append((command_type, cumulitive_dist))
        command_type = 0
        cumulitive_dist = 0
        robot_commands_optimised.append(command)
robot_commands = robot_commands_optimised

print()
print(robot_commands)
print()

data_list = list()

for i in robot_commands:
    data_list.append(i[0])
    data_list.append(i[1])

#print(f"{bin(2)[2::]:0>8}")
#print(start_point)

#print(*field_unmodified, sep = "\n")

def send_data(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))
    print("connected")

    ba = bytearray(struct.pack("i" * len(robot_commands)*2, *data_list))

    s.send(ba)
    print("sended")
    s.close()

send_data('192.168.1.1', 80)

cv2.imshow(window_name, transformed_frame)
cv2.moveWindow(window_name, window_origin[0], window_origin[1])
cv2.waitKey(0)
cv2.destroyAllWindows()