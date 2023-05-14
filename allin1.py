import time  
import RPi.GPIO as GPIO  
import json  
import RGB
import Adafruit_PCA9685  
import cv2
import move as mv
import numpy as np
import GUImove as move
import servo


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
servoLeft = 180     
servoRight = 480 

rangeKeep = 0.3               
scanDir = 1     
scanPos = 1     
scanNum = 3     
scanList = [0,0,0]


vitesseM=85
vitesseL=100
vitesseR=100
vitesseRLM=85
vitesseRM=85
vitesseLM=85
vitesse6=70


timeM=0.20
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





angle_rate=0.8
speed = 70

vitesse=85
angle=300
temps=0.20

cap = cv2.VideoCapture(0)

cap.set(3, 640)
cap.set(4, 480)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output1.avi', fourcc, 20.0, (640, 480))
out1 = cv2.VideoWriter('fleche_all.avi', fourcc, 20.0, (640, 480))

a=1

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

def detectLine(a):
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
            a=2
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
    return a

def detectObject():
    print('Automatic obstacle avoidance mode')  
    if scanPos == 1:  
        pwm.set_pwm(servoPort, 0, servoLeft)  
        time.sleep(0.3)  
        scanList[0] = checkdist()  

    elif scanPos == 2:  
        pwm.set_pwm(servoPort, 0, servoMiddle)  
        time.sleep(0.3)  
        scanList[1] = checkdist()  

    elif scanPos == 3:  
        pwm.set_pwm(servoPort, 0, servoRight)  
        time.sleep(0.3)  
        scanList[2] = checkdist()  

    scanPos = scanPos + scanDir  

    if scanPos > scanNum or scanPos < 1:  

        if scanDir == 1:scanDir = -1  

        elif scanDir == -1:scanDir = 1  

        scanPos = scanPos + scanDir*2  

    print(scanList)  

    if min(scanList) < rangeKeep:  

        if scanList.index(min(scanList)) == 0:  
            print('right')
            pwm.set_pwm(servoportdir,0,angleR)
            mv.move(vitesseR, 'forward', None)
            time.sleep(timeR)

        elif scanList.index(min(scanList)) == 1:    

            if scanList[0] < scanList[2]:  
                print('right')
                pwm.set_pwm(servoportdir,0,angleR)
                mv.move(vitesseR, 'forward', None)
                time.sleep(timeR)

            else:  
                print('left')
                pwm.set_pwm(servoportdir,0,angleL)
                mv.move(vitesseL, 'forward', None)
                time.sleep(timeL)  

        elif scanList.index(min(scanList)) == 2:    
            print('left')
            pwm.set_pwm(servoportdir,0,angleL)
            mv.move(vitesseL, 'forward', None)
            time.sleep(timeL)

        elif max(scanList) < rangeKeep:  
            print('reverse') 
            pwm.set_pwm(servoportdir,0,315)
            mv.move(vitesse6, 'backward', None)
            time.sleep(time6)
            

    else:  
        print('forward')
        pwm.set_pwm(servoportdir,0,angleM)
        mv.move(vitesseM, 'forward', None)
        time.sleep(timeM) 

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

def couleur() :
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
            return 2
        if contient_vert(image)==1 :
            print("vert")
            return 3
            

        
        
        cv2.waitKey(0)
        cap.release()
        cv2.destroyAllWindows()

def fleche() :
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Erreur: Impossible d'ouvrir la caméra")
        exit()
        
    while True:
        
        ret, frame = cap.read()
        
        if not ret:
            break;
    
        
        image1 = frame
        #image2 = cv2.imread('annexe2.jpg')
        
        image1_resize = cv2.resize(image1, (800, 600))
        
        gris = cv2.cvtColor(image1_resize, cv2.COLOR_BGR2GRAY)
        
        
        
        
        
        gris = np.float32(gris)
        
        dst = cv2.cornerHarris(gris,2,3,0.04)      # Harris appliqué
        
        dst = cv2.dilate(dst, None)
        
        gris = cv2.GaussianBlur(gris, (5,5),0)
        
        image1_resize2 = cv2.cvtColor(gris, cv2.COLOR_GRAY2RGB)
        
        seuil = 0.01* dst.max()
        
        image1_resize[dst > seuil] = [0,0,255]
        
        
        #Matrice et fonction goodFeat...
        
        coins = cv2.goodFeaturesToTrack(gris,25,0.01, 10)
        
        print(coins)
        
        for i in coins:
            x,y = i.ravel() # Tableau en une dimension
            
            print("x = " + str(x))
            
            print("y = " + str(y))
            
            cv2.circle(image1_resize, (int(x), int(y)), 20, (0, 255, 0), -1)
        
        min_value = np.min(coins)            # Min et max avec librairie numpy
        max_value = np.max(coins)
        
        print("Le min est : ", min_value)
        print("Le max est : ", max_value)
        
        x_moyenne = (min_value + max_value)/2
        
        print("La moyenne : ", x_moyenne)
        
        cv2.line(image1_resize, (int(x_moyenne), 0), (int(x_moyenne), image1_resize.shape[0]), (0,0,255), 1) # Ligne rouge en fonction de la valeur moyenne precedente
        
        
        gauche=0
        
        droite=0
        
        for i in coins:
        
            x , variable_inutile_mais_fait_mamrcher_le_ravel = i.ravel()
            
            if x_moyenne> int(x)  : # Ligne rouge permet de delimiter gauche / droite
                
                gauche+=1
                
            else:
                
                droite+=1
        
        print("Points à gauche" ,gauche)
        
        print("Points à droite" ,droite)
        
        if (gauche > droite):
        
            print("Tourner à gauche")
            servo.turnLeft(angle_rate)
            
        else:
            print("Tourner à droite")
            servo.turnRight(angle_rate)
            
        
        return(4)
        
        
        cv2.waitKey(0)
        cap.release()
        cv2.destroyAllWindows()




setup()
while a==1:
    a=detectLine(a)

while a==2:
    
        
    pwm.set_pwm(servoPort, 0, 180)
        
    while a==2:
        a=couleur()
            
    time.sleep(1)  
    move.move(speed, 'forward')
    time.sleep(1)

while a==3:
        
    while a==3:
        a=fleche()
            
    time.sleep(1)  
    move.move(speed, 'forward')
    time.sleep(1)
    

while a==4:
    detectObject()