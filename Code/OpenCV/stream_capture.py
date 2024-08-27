import cv2
import numpy as np
import socket

stream = cv2.VideoCapture("rtsp://192.168.22.138:8554/live", cv2.CAP_ANY)

ret, img = stream.read()

cv2.imshow("Img", img)
stream.release()

def send_data(ip, port, values):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))

    ba = bytearray(1)
    s.send(ba)
    
    s.close()

send_data('192.168.1.1', 80, 0)



cv2.waitKey(0)
cv2.destroyAllWindows()