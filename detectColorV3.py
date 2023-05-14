
import cv2 
import numpy as np
import time
import GUImove as move
import servo
import RPi.GPIO as GPIO
import Adafruit_PCA9685

line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20
angle_rate=0.8
speed = 70

servo=1

pwm=Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)
    #motor.setup()

def contient_vert(image):
    
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_green = (36, 25, 25)
    upper_green = (86, 255, 255)
    
    green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
    
    has_green = cv2.countNonZero(green_mask) > 0
    
    return 1 if has_green else 0

def contient_rouge(image):

    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_red1 = (0, 70, 50)
    upper_red1 = (10, 255, 255)
    lower_red2 = (170, 70, 50)
    upper_red2 = (180, 255, 255)
    
    red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    has_red = cv2.countNonZero(red_mask) > 0
    
    return 1 if has_red else 0


def fleche() :
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("1")
        exit()
        
    while True:
        
        ret, frame = cap.read()
        
        if not ret:
            break;
            
        image = frame
        
        if contient_rouge(image)==1 :
            print("rouge")
            return 0
        if contient_vert(image)==1 :
            print("vert")
            return 1
            

        
        
        cv2.waitKey(0)
        cap.release()
        cv2.destroyAllWindows()

    

if __name__ == '__main__':
    try:
        setup()
        move.setup()
        a=0
        
        pwm.set_pwm(servo, 0, 180)
        
        while a==0:
            a=fleche()
            
        time.sleep(1)  
        move.move(speed, 'forward')
        time.sleep(1)
            
        pass
    except KeyboardInterrupt:
        move.destroy()