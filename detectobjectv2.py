import time  
import RPi.GPIO as GPIO  
import json  
import RGB
import Adafruit_PCA9685  
import move as mv


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

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


def setup():
    GPIO.setwarnings(False)
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



setup()
while 1:
    detectObject()