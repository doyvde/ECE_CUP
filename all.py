import time  
import RPi.GPIO as GPIO  
import json  
import RGB
import Adafruit_PCA9685  
import cv2
import move as mv
import numpy as np


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19
Tr = 11          
Ec = 8           

servoPort = 1 
servoportdir = 2      

servoMiddle = 330   
servoLeft = 200     
servoRight = 460 

GPIO.setmode(GPIO.BCM)  

GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)  

GPIO.setup(Ec, GPIO.IN) 

Motor_B_EN    = 17

Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 0
left_backward = 1

right_forward = 0
right_backward= 1


pwm_B = 0

pwm2_direction = 1
pwm2_init = 300
pwm2_range = 150
pwm2_max  = 450
pwm2_min  = 150
pwm2_pos  = pwm2_init

RGB.setup()
RGB.cyan()


vitesseM=65
vitesseL=80
vitesseR=80
vitesseRLM=65
vitesseRM=65
vitesseLM=65
vitesse6=50


timeM=0.10
timeL=0.15
timeR=0.15
timeRLM=0
timeRM=0.25
timeLM=0.25
time6=0.1


angleM=300
angleL=380
angleR=225
angleRLM=300
angleRM=240
angleLM=360
angle6=315

etape=2



vitesse=85
angle=300
temps=0.20

cap = cv2.VideoCapture(0)

cap.set(3, 640)
cap.set(4, 480)



fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('colorall.avi', fourcc, 20.0, (640, 480))




def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)
    mv.setup()

    GPIO.setmode(GPIO.BCM)  
    GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)  
    GPIO.setup(Ec, GPIO.IN)

def checkdist():  
    GPIO.output(Tr, GPIO.HIGH) 
    time.sleep(0.000015)  
    GPIO.output(Tr, GPIO.LOW)  

    while not GPIO.input(Ec): 
        pass  

    t1 = time.time() 

    while GPIO.input(Ec): 
        pass  

    t2 = time.time() 

    return (t2-t1)*340/2  

def detectLine(etape):
    pwm.set_pwm(servoportdir, 0, angleM)
    right = GPIO.input(line_pin_right)
    middle = GPIO.input(line_pin_middle)
    left = GPIO.input(line_pin_left)

    print("Capteur G : ",left, " Capteur M : ",middle," Capteur D : ", right)

    if middle == 1 and left == 0 and right == 0:
        
        pwm.set_pwm(servoportdir,0,angleM)
        mv.move(vitesseM, 'forward', None)
        time.sleep(timeM)
        if right == 1 and left == 1 and middle == 1 :
            pwm.set_pwm(servoportdir,0,angleRLM)
            mv.move(vitesseRLM,'forward',None)
            time.sleep(timeRLM)
            etape=2
    elif left == 1 and middle == 0 and right == 0:
        
        pwm.set_pwm(servoportdir,0,angleL)
        mv.move(vitesseL, 'forward', None)
        time.sleep(timeL)
    elif right == 1 and middle == 0 and left == 0:
        
        pwm.set_pwm(servoportdir,0,angleR)
        mv.move(vitesseR, 'forward', None)
        time.sleep(timeR)
    elif right == 1 and left == 1 and middle == 1 :
        pwm.set_pwm(servoportdir,0,angleRLM)
        mv.move(vitesseRLM,'forward',None)
        time.sleep(timeRLM)
    elif right == 1 and middle == 1 :
        pwm.set_pwm(servoportdir,0,angleRM)
        mv.move(vitesseRM,'forward',None)
        time.sleep(timeRM)
    elif left == 1 and middle == 1:
        pwm.set_pwm(servoportdir,0,angleLM)
        mv.move(vitesseLM,'forward',None)
        time.sleep(timeLM)
    else:
        pwm.set_pwm(servoportdir,0,315)
        mv.move(vitesse6, 'backward', None)
        time.sleep(time6)
    return etape

def detectColor(etape):
    ret, frame = cap.read()

    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 5)
    
    lower_red = (0, 50, 50)
    upper_red = (10, 255, 255)
    mask1 = cv2.inRange(hsv_frame, lower_red, upper_red)

    lower_red = (170, 50, 50)
    upper_red = (180, 255, 255)
    mask2 = cv2.inRange(hsv_frame, lower_red, upper_red)

    
    lower_green = (50, 100, 100)
    upper_green = (70, 255, 255)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)

    
    mask = cv2.bitwise_or(mask1, mask2)
    red_masked_frame = cv2.bitwise_and(mask, mask, mask=thresh)
    green_masked_frame = cv2.bitwise_and(mask_green, mask_green, mask=thresh)

    
    contours_red = cv2.findContours(red_masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    contours_green = cv2.findContours(green_masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    
    for contour_red in contours_red:
        xr, yr, wr, hr = cv2.boundingRect(contour_red)
        cv2.rectangle(frame, (xr, yr), (xr+wr, yr+hr), (0, 0, 255), 2)

    for contour_green in contours_green:
        xg, yg, wg, hg = cv2.boundingRect(contour_green)
        cv2.rectangle(frame, (xg, yg), (xg+wg, yg+hg), (0, 255, 0), 2)
    
    

    if cv2.countNonZero(mask) > 0:
        print("R:1")
        print('Stop')
        mv.motorStop()
        out.write(frame)
        etape=2
        

    else:
        print("R:0")
        print('forward')
        pwm.set_pwm(servoportdir,0,angle)
        mv.move(vitesse, 'forward', None)
        time.sleep(0.5)
        mv.motorStop()
        out.write(frame)
        etape=3
        

    return etape

def detectForme(etape):
    ret, img_resized = cap.read()

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
        out1.write(img_resized)
        etape=4
        
        

    else:
        print("flèche vers la gauche")
        print('forward')
        pwm.set_pwm(servoportdir,0,angleL)
        mv.move(vitesse, 'forward', None)
        time.sleep(temps)
        out1.write(img_resized)
        etape=4
        

    return etape

def detectObject():

     rangeKeep = 0.3               
     scanDir = 1     
     scanPos = 1     
     scanNum = 3     
     scanList = [0,0,0]
     print('Automatic obstacle avoidance mode')  

       

     pwm.set_pwm(servoPort, 0, servoLeft)  

     time.sleep(0.3)  

     scanList[0] = checkdist()  

       

     pwm.set_pwm(servoPort, 0, servoMiddle)  

     time.sleep(0.3)  

     scanList[1] = checkdist()  

       

     pwm.set_pwm(servoPort, 0, servoRight)  

     time.sleep(0.3)  

     scanList[2] = checkdist()  


     print(scanList)  

    

     if min(scanList) < rangeKeep:  

         if scanList.index(min(scanList)) == 0:  #The shortest distance detected on the left

            ''' 

            Turn right 

             '''  
            speed_set = 80
            setup()
            mv.move(speed_set, 'forward', 'right', 0.8)
            print('Turn right') 
            time.sleep(0.3)
            mv.motorStop()
              

         elif scanList.index(min(scanList)) == 1:    #The shortest distance detected in the middle

             if scanList[0] < scanList[2]:  

                ''' 

                 If the detected distance on the left is shorter than the right, turn to the right

                 '''  
                speed_set = 80
                setup()
                mv.move(speed_set, 'forward', 'right', 0.8)
                print('Turn right') 
                time.sleep(0.3)
                mv.motorStop()

             else:  

                 ''''' 

                 Otherwise, turn left 

                 '''  
                 speed_set = 80
                 setup()
                 mv.move(speed_set, 'forward', 'left', 0.8)
                 time.sleep(0.3)
                 mv.motorStop()
                 print('Turn left')  

         elif scanList.index(min(scanList)) == 2:    #La distance la plus courte détectée à droite
             ''' 

             Turn Left 

             '''  
             speed_set = 80
             setup()
             mv.move(speed_set, 'forward', 'left', 0.8)
             time.sleep(0.3)
             mv.motorStop()
             print('Turn Left')  

         elif max(scanList) < rangeKeep:  

             ''' 

             If the distances in the left, center, and right directions are all closer than rangeKeep, reverse 

             '''  
             speed_set = 80
             setup()
             mv.move(speed_set, 'backward', 'no', 0.8)
             time.sleep(0.3)
             mv.motorStop()
             print('reverse')  

     else:  

         ''' 

         All three directions are farther than rangeKeep

         '''  
         speed_set = 80
         setup()
         mv.move(speed_set, 'forward', 'no', 0.8)
         time.sleep(0.3)
         mv.motorStop()
         print('Go forward')



setup()
while etape==1:

    etape=detectLine(etape)

while etape==2:

    pwm.set_pwm(servoPort, 0, 250)
    etape=detectColor(etape)
    
pwm.set_pwm(servoPort, 0, 330)

cap.release()

time.sleep(1)

cap = cv2.VideoCapture(0)

cap.set(3, 640)
cap.set(4, 480)

out1 = cv2.VideoWriter('fleche_all.avi', fourcc, 20.0, (640, 480))

while etape==3:

    
    etape=detectForme(etape)
    

while etape==4: 
    cap.release()
    out1.release()
    cv2.destroyAllWindows()
    detectObject()
