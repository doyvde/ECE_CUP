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

objects=0 

rangeKeep = 0.4     #Avoidance distance 

rangeKeep2 = 0.1     #Avoidance distance            

scanDir = 1     #Scan direction, 1 is from left to right, -1 is from right to left

scanPos = 1     #Store the current scan position (1 is the left, 2 is the middle, and 3 is the right)

scanNum = 3     #The number of scan positions (left, middle, and right, these are three positions)

scanList = [0,0,0]  #Save scan results 
   
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


def turnMiddle():
        global pwm2_pos
        pwm2_pos = pwm2_init
        RGB.both_off()
        RGB.red()
        pwm.set_pwm(2, 0, pwm2_pos)

def motorStop():#Motor stops
        
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)

def setup():#Motor initialization
        global pwm_B
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_B_EN, GPIO.OUT)
        GPIO.setup(Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(Motor_B_Pin2, GPIO.OUT)

        motorStop()
        try:
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
                        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                        GPIO.output(Motor_B_Pin2, GPIO.LOW)
                        pwm_B.start(100)
                        pwm_B.ChangeDutyCycle(speed)
                        
                elif direction == Dir_forward:
                        turnMiddle()
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
                        RGB.turn_left(3)
                        pwm.set_pwm(2, 0, 225)
                        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                        GPIO.output(Motor_B_Pin2, GPIO.LOW)
                        pwm_B.start(100)
                        pwm_B.ChangeDutyCycle(speed)
                        
                elif direction == Dir_forward:
                        RGB.turn_left(3)
                        pwm.set_pwm(2, 0, 225)
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
                        RGB.turn_right(3)
                        pwm.set_pwm(2, 0, 375)
                        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                        GPIO.output(Motor_B_Pin2, GPIO.LOW)
                        pwm_B.start(100)
                        pwm_B.ChangeDutyCycle(speed)
                        
                elif direction == Dir_backward:
                        RGB.turn_right(3)
                        pwm.set_pwm(2, 0, 375)
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
                        motor(1, Dir_forward, speed)
                        
        elif direction == 'backward':
                    if turn == 'right':
                            motor_right(1, right_backward, speed)
                            
                    elif turn == 'left':
                            motor_left(1, left_backward, speed)
                            
                    else:
                            motor(1, Dir_backward, speed)
                            
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

   

     while not GPIO.input(Ec): # When the module no longer receives the initial sound wave 

         pass  

     t1 = time.time() # Write down the time when the initial sound wave was emitted. 

     while GPIO.input(Ec): # When the module receives the return sound wave.

         pass  

     t2 = time.time() # Write down the time when the return sound wave was captured.

   

     return (t2-t1)*340/2 # Calculate the distance. 
   

    
while objects == 0 :  
    
         print('Automatic obstacle avoidance mode')  
    
           
    
         pwm.set_pwm(servoPort, 0, servoMiddle)  
    
         time.sleep(0.3)  
    
         scanLists = checkdist()  
    
           
    
    
         print(scanLists)  
    
       
    
         if scanLists < 0.4:  
     
            speed_set = 50
            setup()
            RGB.turn_left(3)
            pwm.set_pwm(2, 0, 410)
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(75)
            time.sleep(3)
            motorStop()
            print('Turn left')
        
            objects=objects+1
             
    
         else:  
    
             ''' 
    
             All three directions are farther than rangeKeep
    
             '''  
             speed_set = 80
             setup()
             move(speed_set, 'forward', 'no', 0.8)
             time.sleep(0.3)
             motorStop()
             print('Go forward')  
    
while objects == 1 :  
    
         print('Automatic obstacle avoidance mode 2')  
    
           
    
         pwm.set_pwm(servoPort, 0, servoMiddle)  
    
         time.sleep(0.3)  
    
         scanLists = checkdist()  
    
           
    
    
         print(scanLists)  
    
       
    
         if scanLists < 0.15:  
             
            motorStop()
            print('STOP') 
            
            objects=objects+1
            
         else:  
    
             ''' 
    
             All three directions are farther than rangeKeep
    
             '''  
             speed_set = 65
             setup()
             move(speed_set, 'forward', 'no', 0.8)
             time.sleep(0.3)
             motorStop()
             print('Go forward')
while objects == 2 : 
    
    print('Automatic obstacle avoidance mode 3')  

      

    pwm.set_pwm(servoPort, 0, 180)  

    time.sleep(0.3)  

    scanLists = checkdist() 
    
    if scanLists > 0.45:  
        
        speed_set = 65
        setup()
        move(speed_set, 'backward', 'no', 0.8)
        time.sleep(0.85)
        motorStop()
        print('Go backward')
        speed_set = 50
        setup()
        RGB.turn_right(3)
        pwm.set_pwm(2, 0, 230)
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        pwm_B.start(100)
        pwm_B.ChangeDutyCycle(80)
        time.sleep(7)
        motorStop()
        print('Turn right')
           
        objects=objects+1
       
    else:  

        ''' 

        All three directions are farther than rangeKeep

        '''  
        speed_set = 65
        setup()
        move(speed_set, 'backward', 'no', 0.8)
        time.sleep(0.3)
        motorStop()
        print('Go backward')
    
while objects == 3 :  
    
         print('Automatic obstacle avoidance mode4')  
    
           
    
         pwm.set_pwm(servoPort, 0, 480)  
    
         time.sleep(0.3)  
    
         scanLists = checkdist()  
    
           
    
    
         print(scanLists)  
    
       
    
         if scanLists > 0.45:  

            motorStop()
            print('STOP')
        
            objects=objects+1
             
    
         else:  
    
             ''' 
    
             All three directions are farther than rangeKeep
    
             '''  
             speed_set = 80
             setup()
             move(speed_set, 'forward', 'no', 0.8)
             time.sleep(0.3)
             motorStop()
             print('Go forward') 