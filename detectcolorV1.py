import cv2


cap = cv2.VideoCapture(0)


cap.set(3, 640)
cap.set(4, 480)


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))


while True:
    
    ret, frame = cap.read()

    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    
    lower_red = (0, 50, 50)
    upper_red = (10, 255, 255)
    mask1 = cv2.inRange(hsv_frame, lower_red, upper_red)

    lower_red = (170, 50, 50)
    upper_red = (180, 255, 255)
    mask2 = cv2.inRange(hsv_frame, lower_red, upper_red)

    
    mask = mask1 + mask2

    
    contours= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

    
    out.write(frame)

    

    
    if cv2.waitKey(1) == ord('q'):
        break


cap.release()
out.release()
cv2.destroyAllWindows()