import time
import RPi.GPIO as GPIO
import move as mv
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19
servoportdir=2

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

def setupline():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)
    mv.setup()

def detectline():
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

setupline()
while 1:
    detectline()
