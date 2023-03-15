import time  

import RPi.GPIO as GPIO  

import json  

import RGB

import Adafruit_PCA9685  
  
pwm = Adafruit_PCA9685.PCA9685()  

pwm.set_pwm_freq(50)  

Tr = 11         #Pin No. of Ultrasonic Module Input  

Ec = 8          # Pin number of the output end of the ultrasonic module 

servoPort = 1       #The number of the servo that controls the horizontal rotation of the ultrasonic module 

servoMiddle = 330   #The middle position of the servo 

servoLeft = 180     #Left position of the servo

servoRight = 480    #The right position of the servo


servoPort2 = 2       #The number of the servo that controls the horizontal rotation of the ultrasonic module 


rangeKeep = 0.3     #Avoidance distance            

scanDir = 1     #Scan direction, 1 is from left to right, -1 is from right to left

scanPos = 1     #Store the current scan position (1 is the left, 2 is the middle, and 3 is the right)

scanNum = 3     #The number of scan positions (left, middle, and right, these are three positions)

scanList = [0,0,0]  #Save scan results 
   
GPIO.setmode(GPIO.BCM)  

GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)  

GPIO.setup(Ec, GPIO.IN)  

Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 14
Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 0
left_backward = 1

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

pwm2_direction = 1
pwm2_init = 300
pwm2_range = 150
pwm2_max  = 450
pwm2_min  = 150
pwm2_pos  = pwm2_init

RGB.setup()
RGB.cyan()

def ctrl_range(raw, max_genout, min_genout):
	if raw > max_genout:
		raw_output = max_genout
	elif raw < min_genout:
		raw_output = min_genout
	else:
		raw_output = raw
	return int(raw_output)

def turnLeft(coe=1):
        global pwm2_pos
        pwm2_pos = pwm2_init + int(coe*pwm2_range*pwm2_direction)
        pwm2_pos = ctrl_range(pwm2_pos, pwm2_max, pwm2_min)
        RGB.turn_left(3)
        #RGB.yellow()
        pwm.set_pwm(2, 0, 225)


def turnRight(coe=1):
        global pwm2_pos
        pwm2_pos = pwm2_init - int(coe*pwm2_range*pwm2_direction)
        pwm2_pos = ctrl_range(pwm2_pos, pwm2_max, pwm2_min)
        RGB.turn_right(3)
        #RGB.yellow()
        pwm.set_pwm(2, 0, 375)


def turnMiddle():
        global pwm2_pos
        pwm2_pos = pwm2_init
        RGB.both_on()
        pwm.set_pwm(2, 0, pwm2_pos)

def motorStop():#Motor stops
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
        global pwm_A, pwm_B
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_A_EN, GPIO.OUT)
        GPIO.setup(Motor_B_EN, GPIO.OUT)
        GPIO.setup(Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(Motor_B_Pin2, GPIO.OUT)

        motorStop()
        try:
                pwm_A = GPIO.PWM(Motor_A_EN, 1000)
                pwm_B = GPIO.PWM(Motor_B_EN, 1000)
        except:
                pass

def motor(status, direction, speed):#Motor 2 positive and negative rotation
        if status == 0: # stop
        
                GPIO.output(Motor_B_Pin1, GPIO.LOW)
                GPIO.output(Motor_B_Pin2, GPIO.LOW)
                GPIO.output(Motor_B_EN, GPIO.LOW)
        else:
                if direction == Dir_backward:
                        turnMiddle()
                        #pwm.set_pwm(servoPort2, 0, servoMiddle2)
                        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                        GPIO.output(Motor_B_Pin2, GPIO.LOW)
                        pwm_B.start(100)
                        pwm_B.ChangeDutyCycle(speed)
                        
                elif direction == Dir_forward:
                        turnMiddle()
                        #pwm.set_pwm(servoPort2, 0, servoMiddle2)
                        GPIO.output(Motor_B_Pin1, GPIO.LOW)
                        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
                        pwm_B.start(0)
                        pwm_B.ChangeDutyCycle(speed)

def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
        if status == 0: # stop
        
                GPIO.output(Motor_B_Pin1, GPIO.LOW)
                GPIO.output(Motor_B_Pin2, GPIO.LOW)
                GPIO.output(Motor_B_EN, GPIO.LOW)
        else:
                if direction == Dir_backward:
                        turnLeft()
                        #pwm.set_pwm(servoPort2, 0, servoLeft2)
                        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                        GPIO.output(Motor_B_Pin2, GPIO.LOW)
                        pwm_B.start(100)
                        pwm_B.ChangeDutyCycle(speed)
                        
                elif direction == Dir_forward:
                        turnLeft()
                        #pwm.set_pwm(servoPort2, 0, servoLeft2)
                        GPIO.output(Motor_B_Pin1, GPIO.LOW)
                        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
                        pwm_B.start(0)
                        pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
        if status == 0: # stop
                GPIO.output(Motor_B_Pin1, GPIO.LOW)
                GPIO.output(Motor_B_Pin2, GPIO.LOW)
                GPIO.output(Motor_B_EN, GPIO.LOW)
        else:
                if direction == Dir_forward:#
                        turnRight()
                        #pwm.set_pwm(servoPort2, 0, servoRight2)
                        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                        GPIO.output(Motor_B_Pin2, GPIO.LOW)
                        pwm_B.start(100)
                        pwm_B.ChangeDutyCycle(speed)
                        
                elif direction == Dir_backward:
                        turnRight()
                        #pwm.set_pwm(servoPort2, 0, servoRight2)
                        GPIO.output(Motor_B_Pin1, GPIO.LOW)
                        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
                        pwm_B.start(0)
                        pwm_B.ChangeDutyCycle(speed)
        return direction


def move(speed, direction, turn, radius=0.6):   # 0 < radius <= 1
        #speed = 100
        if direction == 'forward':
                if turn == 'right':
                        motor_right(1, right_forward, speed)
                        
                elif turn == 'left':
                        motor_left(1, left_forward, speed)
                        
                else:
                        motor(1, left_forward, speed)
                        
        elif direction == 'backward':
                    if turn == 'right':
                            motor_right(1, right_backward, speed)
                            
                    elif turn == 'left':
                            motor_left(1, left_backward, speed)
                            
                    else:
                            motor(1, left_backward, speed)
                            
        elif direction == 'no':
                    if turn == 'right':
                            motor_right(1, right_forward, speed)
                            
                    elif turn == 'left':
                            motor_left(1, left_forward, speed)
                            
                    else:
                            motorStop()
        else:
                    pass
                
                
                
def destroy():
        motorStop()
        GPIO.cleanup()             # Release resource 



def checkdist():  

     ''' Refer to the realization of basic functions-ultrasonic module '''  
     GPIO.output(Tr, GPIO.HIGH) # Set the input terminal of the module to high level and send out an initial sound wave.  
     time.sleep(0.000015)  

     GPIO.output(Tr, GPIO.LOW)  

   

     while not GPIO.input(Ec): # When the module no longer receives the initial sound wave ①

         pass  

     t1 = time.time() # Write down the time when the initial sound wave was emitted. 

     while GPIO.input(Ec): # When the module receives the return sound wave.

         pass  

     t2 = time.time() # Write down the time when the return sound wave was captured.

   

     return (t2-t1)*340/2 # Calculate the distance. 
   

while 1:  

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

            ''' 

            Turn right 

             '''  
            speed_set = 80
            setup()
            move(speed_set, 'forward', 'right', 0.8)
            print('Turn right') 
            time.sleep(1.3)
            motorStop()
              

         elif scanList.index(min(scanList)) == 1:    #The shortest distance detected in the middle

             if scanList[0] < scanList[2]:  

                ''' 

                 If the detected distance on the left is shorter than the right, turn to the right

                 '''  
                speed_set = 80
                setup()
                move(speed_set, 'forward', 'right', 0.8)
                print('Turn right') 
                time.sleep(1.3)
                motorStop()

             else:  

                 ''''' 

                 Otherwise, turn left 

                 '''  
                 speed_set = 80
                 setup()
                 move(speed_set, 'forward', 'left', 0.8)
                 time.sleep(1.3)
                 motorStop()
                 print('Turn left')  

         elif scanList.index(min(scanList)) == 2:    #The shortest distance detected on the right 

             ''' 

             Turn Left 

             '''  
             speed_set = 80
             setup()
             move(speed_set, 'forward', 'left', 0.8)
             time.sleep(1.3)
             motorStop()
             print('Turn Left')  

         elif max(scanList) < rangeKeep:  

             ''' 

             If the distances in the left, center, and right directions are all closer than rangeKeep, reverse 

             '''  
             speed_set = 80
             setup()
             move(speed_set, 'backward', 'no', 0.8)
             time.sleep(1.3)
             motorStop()
             print('reverse')  

     else:  

         ''' 

         All three directions are farther than rangeKeep

         '''  
         speed_set = 80
         setup()
         move(speed_set, 'forward', 'no', 0.8)
         time.sleep(1.3)
         motorStop()
         print('Go forward')  