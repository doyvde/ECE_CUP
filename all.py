import time  
import RPi.GPIO as GPIO  
import json  
import RGB
import Adafruit_PCA9685  
import cv2
import move as mv


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19
Tr = 11         #Pin No. of Ultrasonic Module Input  
Ec = 8          # Pin number of the output end of the ultrasonic module 

servoPort = 1 
servoportdir = 2      #The number of the servo that controls the horizontal rotation of the ultrasonic module 

servoMiddle = 330   #The middle position of the servo 
servoLeft = 180     #Left position of the servo
servoRight = 480 

rangeKeep = 0.3     #Avoidance distance            
scanDir = 1     #Scan direction, 1 is from left to right, -1 is from right to left
scanPos = 1     #Store the current scan position (1 is the left, 2 is the middle, and 3 is the right)
scanNum = 3     #The number of scan positions (left, middle, and right, these are three positions)
scanList = [0,0,0]

#parametre speed
vitesseM=85
vitesseL=100
vitesseR=100
vitesseRLM=85
vitesseRM=85
vitesseLM=85
vitesse6=70

#parametre time
timeM=0.20
timeL=0.15
timeR=0.15
timeRLM=0
timeRM=0.25
timeLM=0.25
time6=0.1

#parametre angle
angleM=300
angleL=380
angleR=225
angleRLM=300
angleRM=240
angleLM=360
angle6=315


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
    GPIO.output(Tr, GPIO.HIGH) # Set the input terminal of the module to high level and send out an initial sound wave.  
    time.sleep(0.000015)  
    GPIO.output(Tr, GPIO.LOW)  

    while not GPIO.input(Ec): # When the module no longer receives the initial sound wave 
        pass  

    t1 = time.time() # Write down the time when the initial sound wave was emitted. 

    while GPIO.input(Ec): # When the module receives the return sound wave.
        pass  

    t2 = time.time() # Write down the time when the return sound wave was captured.

    return (t2-t1)*340/2 # Calculate the distance. 

def detectLine():
    pwm.set_pwm(servoportdir, 0, angleM)
    right = GPIO.input(line_pin_right)
    middle = GPIO.input(line_pin_middle)
    left = GPIO.input(line_pin_left)
    
    print("Capteur G : ",left, " Capteur M : ",middle," Capteur D : ", right)

    if middle == 1 and left == 0 and right == 0:# 0 --> white line / 1 --> black line
        #print('forward')
        pwm.set_pwm(servoportdir,0,angleM)
        mv.move(vitesseM, 'forward', None)
        time.sleep(timeM)
    elif left == 1 and middle == 0 and right == 0:# 0 --> white line / 1 --> black line
        #print('left')
        pwm.set_pwm(servoportdir,0,angleL)
        mv.move(vitesseL, 'forward', None)
        time.sleep(timeL)
    elif right == 1 and middle == 0 and left == 0:# 0 --> white line / 1 --> black line
        #print('right')
        pwm.set_pwm(servoportdir,0,angleR)
        mv.move(vitesseR, 'forward', None)
        time.sleep(timeR)
    elif right == 1 and left == 1 and middle == 1 :# 0 --> white line / 1 --> black line
        pwm.set_pwm(servoportdir,0,angleRLM)
        mv.move(vitesseRLM,'forward',None)
        time.sleep(timeRLM)
    elif right == 1 and middle == 1 :# 0 --> white line / 1 --> black line
        pwm.set_pwm(servoportdir,0,angleRM)
        mv.move(vitesseRM,'forward',None)
        time.sleep(timeRM)
    elif left == 1 and middle == 1:# 0 --> white line / 1 --> black line
        pwm.set_pwm(servoportdir,0,angleLM)
        mv.move(vitesseLM,'forward',None)
        time.sleep(timeLM)
    else:
        pwm.set_pwm(servoportdir,0,315)
        mv.move(vitesse6, 'backward', None)
        time.sleep(time6)

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

        if scanList.index(min(scanList)) == 0:  #The shortest distance detected on the left
            print('right')
            pwm.set_pwm(servoportdir,0,angleR)
            mv.move(vitesseR, 'forward', None)
            time.sleep(timeR)

        elif scanList.index(min(scanList)) == 1:    #The shortest distance detected in the middle

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

        elif scanList.index(min(scanList)) == 2:    #The shortest distance detected on the right 
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



setup()
while 1:
    detectLine()