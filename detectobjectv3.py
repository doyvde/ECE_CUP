'''
##########################################################################################################################
######################################################## LIBRAIRIE #######################################################
##########################################################################################################################
'''

import time  

import RPi.GPIO as GPIO  

import json  

import RGB

import Adafruit_PCA9685 
 
'''
##########################################################################################################################
#################################################### DECLARATION VARIABLE ################################################
##########################################################################################################################
 ''' 
 
pwm = Adafruit_PCA9685.PCA9685()  

pwm.set_pwm_freq(50)  

Tr = 11         #Pin No. of Ultrasonic Module Input  

Ec = 8          # Pin number of the output end of the ultrasonic module 

servoPort = 1       #The number of the servo that controls the horizontal rotation of the ultrasonic module 

servoMiddle = 330   #The middle position of the servo 

servoLeft = 180     #Left position of the servo

servoRight = 480    #The right position of the servo

objects=0            
   
GPIO.setmode(GPIO.BCM)  

GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)  

GPIO.setup(Ec, GPIO.IN)  

Motor_EN    = 17

Motor_Pin1  = 27
Motor_Pin2  = 18

pwm_B = 0

RGB.setup()
RGB.cyan()

'''
##########################################################################################################################
###################################################### SOUS-PROGRAMME ####################################################
##########################################################################################################################
'''

def avancer(vitesse,temps):#Motor en avant
        
        GPIO.output(Motor_Pin1, GPIO.LOW)
        GPIO.output(Motor_Pin2, GPIO.HIGH)
        pwm_B.start(100)
        pwm_B.ChangeDutyCycle(vitesse)
        time.sleep(temps)#pause
        motorStop()

def reculer(vitesse,temps): #Motor en arriere
        
        GPIO.output(Motor_Pin2, GPIO.LOW)
        GPIO.output(Motor_Pin1, GPIO.HIGH)
        pwm_B.start(100)
        pwm_B.ChangeDutyCycle(vitesse)
        time.sleep(temps)#pause
        motorStop()

def motorStop(): #Motor stops
        
        GPIO.output(Motor_Pin1, GPIO.LOW)
        GPIO.output(Motor_Pin2, GPIO.LOW)
        GPIO.output(Motor_EN, GPIO.LOW)

def setup():#Motor initialization
        global pwm_B
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_EN, GPIO.OUT)
        GPIO.setup(Motor_Pin1, GPIO.OUT)
        GPIO.setup(Motor_Pin2, GPIO.OUT)

        motorStop()
        try:
                pwm_B = GPIO.PWM(Motor_EN, 1000)
        except:
                pass
            
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
   
'''    
##########################################################################################################################
##################################################### DEBUT DU PROGRAMME #################################################
##########################################################################################################################
'''  
 
while objects == 0 :  #boucle detection premier obstacle

         print('Automatic obstacle avoidance mode')  #affichage dans le treminal pour savoir dans quel boucle on est
         pwm.set_pwm(servoPort, 0, servoMiddle) #positionnement de la tete du robot au millieu 
         time.sleep(0.3) #pause
         scanLists = checkdist()  
         print(scanLists)  
         
         if scanLists < 0.4:  # test de la distance entre le robot et l'obstacle
            setup()
            RGB.turn_left(3)
            pwm.set_pwm(2, 0, 410)#positionnement de la tete du robot au millieu
            avancer(100, 1)
            print('Turn left')
        
            objects=objects+1
             
         else:  
             
             #All three directions are farther than rangeKeep
             setup()
             avancer(80, 0.3)
             print('Go forward')  
    
while objects == 1 :  #boucle detection second obstacle
    
         print('Automatic obstacle avoidance mode 2')  #affichage dans le treminal pour savoir dans quel boucle on est
         pwm.set_pwm(servoPort, 0, servoMiddle)#positionnement de la tete du robot au millieu  
         time.sleep(0.3)  #pause
         scanLists = checkdist()
         print(scanLists)  
    
         if scanLists < 0.15:  
            motorStop()
            print('STOP') 
            
            objects=objects+1
            
         else:  
    
             #All three directions are farther than rangeKeep
             setup()
             avancer(65, 0.3)
             print('Go forward')
             
while objects == 2 : #boucle detection de où se trouve le trou 
    
    print('Automatic obstacle avoidance mode 3')  #affichage dans le treminal pour savoir dans quel boucle on est
    pwm.set_pwm(servoPort, 0, 180) #positionnement de la tete du robot au millieu 
    time.sleep(0.3)  
    scanLists = checkdist() 
    
    if scanLists > 0.35:  
        setup()
        reculer(65, 0.3)
        print('Go backward')
        
        setup()
        RGB.turn_right(3)
        pwm.set_pwm(2, 0, 190)#positionnement de la tete du robot au millieu
        avancer(100, 1)
        print('Turn right')
           
        objects=objects+1
       
    else:  

        #All three directions are farther than rangeKeep
        setup()
        reculer(65, 0.3)
        print('Go backward')
    
while objects == 3 :  #boucle detection d'un obstacle pour la fin on aurait pu juste mis un chrono pour gerer la distance 
    
         print('Automatic obstacle avoidance mode4')  #affichage dans le treminal pour savoir dans quel boucle on est
         pwm.set_pwm(servoPort, 0, servoMiddle)#positionnement de la tete du robot au millieu  
         time.sleep(0.3) #pause 
         scanLists = checkdist()  
         print(scanLists)  
    
         if scanLists < 0.15:  
            motorStop()
            print('Turn right')
        
            objects=objects+1
             
         else:  
    
             #All three directions are farther than rangeKeep
             setup()
             avancer(80, 0.3)
             print('Go forward')    
    
    