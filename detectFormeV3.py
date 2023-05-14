
import cv2 
import numpy as np
import time
import GUImove as move
import servo
import RPi.GPIO as GPIO

line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20
angle_rate=0.8
speed = 70

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)
    #motor.setup()


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
        
      
      return(1)
      
      
      cv2.waitKey(0)
      cap.release()
      cv2.destroyAllWindows()
      
      
    

if __name__ == '__main__':
    try:
        setup()
        move.setup()
        a=0
        
        while a==0:
            a=fleche()
            
        time.sleep(1)  
        move.move(speed, 'forward')
        time.sleep(1)
            
        pass
    except KeyboardInterrupt:
        move.destroy()