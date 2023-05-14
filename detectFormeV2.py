#Q10
import cv2
import numpy as np
import cv2
import time  
import RPi.GPIO as GPIO  
import json  
import RGB
import Adafruit_PCA9685  
import move as mv

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

servoportdir = 2

vitesse=85
angleL=380
angleR=225
temps=0.20

capture = cv2.VideoCapture(0)

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('fleche.avi', fourcc, 20.0, (800, 600))

def setup():
    GPIO.setwarnings(False)
    mv.setup()

    GPIO.setmode(GPIO.BCM)  

def detectForme():
    ret, img_resized = capture.read()

    gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)

    gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    dst = cv2.cornerHarris(gray_blurred, 2, 3, 0.04)

    dst = cv2.dilate(dst, None)

    img_resized[dst > 0.01 * dst.max()] = [0, 0, 255] 

    corners = cv2.goodFeaturesToTrack(gray_blurred, 10, 0.1, 10)

    corners = np.int0(corners)
    corner_matrix = []
    for corner in corners:
        x, y = corner.ravel()
        corner_matrix.append([x, y])
        cv2.circle(img_resized, (x, y), 5, (0, 255, 0), -1)
        print("x :", x, " y :", y)

    X = np.array(corner_matrix)[:,0]
    Y = np.array(corner_matrix)[:,1]
    Xmax_idx = np.argmax(X)
    Xmin_idx = np.argmin(X)
    Xmax = corner_matrix[Xmax_idx][0]
    Ymax = corner_matrix[Xmax_idx][1]
    Xmin = corner_matrix[Xmin_idx][0]
    Ymin = corner_matrix[Xmin_idx][1]

    Xmoyenne = (Xmax + Xmin) // 2

    cv2.line(img_resized, (Xmoyenne, 0), (Xmoyenne, img_resized.shape[0]), (0, 0, 255), 2)

    left_count = 0
    right_count = 0
    for corner in corner_matrix:
        if corner[0] < Xmoyenne:
            left_count += 1
        else:
            right_count += 1

    print("Nombre de sommets à gauche :", left_count)
    print("Nombre de sommets à droite :", right_count)

    if right_count > left_count:
        print("flèche vers la droite")
        print('forward')
        pwm.set_pwm(servoportdir,0,angleR)
        mv.move(vitesse, 'forward', None)
        time.sleep(temps)
        out.write(img_resized)
        

    else:
        print("flèche vers la gauche")
        print('forward')
        pwm.set_pwm(servoportdir,0,angleL)
        mv.move(vitesse, 'forward', None)
        time.sleep(temps)
        out.write(img_resized)


setup()
while True:
    detectForme()
    if cv2.waitKey(1) == ord('q'):
        break

out.release()
capture.release()
cv2.destroyAllWindows()