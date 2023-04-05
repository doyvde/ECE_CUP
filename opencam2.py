import os  

import cv2  

from base_camera import BaseCamera  

import numpy as np  

   

''''' 

Configure target color and HSV color space 

'''  

colorUpper = np.array([44, 255, 255])  

colorLower = np.array([24, 100, 100])  

   

font = cv2.FONT_HERSHEY_SIMPLEX  

   

class Camera(BaseCamera):  
    video_source = 0  

   

def __init__(self):  

    if os.environ.get('OPENCV_CAMERA_SOURCE'):  

        Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))  

    super(Camera, self).__init__()  

@staticmethod  
def set_video_source(source):  

    Camera.video_source = source  

   

@staticmethod  
def frames():  
    camera = cv2.VideoCapture(Camera.video_source)  
    if not camera.isOpened():  
        raise RuntimeError('Could not start camera.')  
    while True:  
    # read current frame  
    _, img = camera.read() #Obtain images captured by the camera  

35.   

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #Transfrom the images to HSV color space   

37.             mask = cv2.inRange(hsv, colorLower, colorUpper) #Loop to detect the color based on the target color range in the HSV color space, and turn the color blocks into masks  

38.             mask = cv2.erode(mask, None, iterations=2)  #Erode and diminish the small masks (hot pixels) in the image (eliminate small color blocks or hot pixels)  

39.             mask = cv2.dilate(mask, None, iterations=2) #Dilate, to resize the large masks eroded in the previous line to the original  

40.             cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,  

41.                 cv2.CHAIN_APPROX_SIMPLE)[-2]            #Find masks in the image  

42.             center = None         

43.             if len(cnts) > 0:   #If the number of masks is more than 1,  

44.                 ''''' 

45.                 Find the coordinate of the center and size of the target color object in the image

46.                 '''  

47.                 c = max(cnts, key=cv2.contourArea)  

48.                 ((box_x, box_y), radius) = cv2.minEnclosingCircle(c)  

49.                 M = cv2.moments(c)  

50.                 center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  

51.                 X = int(box_x)  

52.                 Y = int(box_y)  

53.                 ''''' 

54.                 Obtain and output the coordinate of the center of the target color object

55.                 '''  

56.                 print('Target color object detected')  

57.                 print('X:%d'%X)  

58.                 print('Y:%d'%Y)  

59.                 print('-------')  

60.   

61.                 ''''' 

62.                 Show the text "Target Detected" in the image 

63.                 '''  

64.                 cv2.putText(img,'Target Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)  

65.                 ''''' 

66.                 Mark the target with a frame   

67.                 '''  

68.                 cv2.rectangle(img,(int(box_x-radius),int(box_y+radius)),  

69.                               (int(box_x+radius),int(box_y-radius)),(255,255,255),1)  

70.             else:  

71.                 cv2.putText(img,'Target Detecting',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)  

72.                 print('No target color object detected')  

73.               

74.             # encode as a jpeg image and return it  

75.             yield cv2.imencode('.jpg', img)[1].tobytes()  