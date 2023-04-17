import cv2


cap = cv2.VideoCapture(0)


cap.set(3, 640)
cap.set(4, 480)


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output1.avi', fourcc, 20.0, (640, 480))


while True:
    
    ret, frame = cap.read()

    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 5)
    
    lower_red = (0, 50, 50)
    upper_red = (10, 255, 255)
    mask1 = cv2.inRange(hsv_frame, lower_red, upper_red)

    lower_red = (170, 50, 50)
    upper_red = (180, 255, 255)
    mask2 = cv2.inRange(hsv_frame, lower_red, upper_red)

    
    lower_green = (25, 52, 72)
    upper_green = (102, 255, 255)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)

    
    mask = cv2.bitwise_or(mask1, mask2)
    red_masked_frame = cv2.bitwise_and(mask, mask, mask=thresh)
    green_masked_frame = cv2.bitwise_and(mask_green, mask_green, mask=thresh)

    
    contours_red = cv2.findContours(red_masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    contours_green = cv2.findContours(green_masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    
    for contour_red in contours_red:
        xr, yr, wr, hr = cv2.boundingRect(contour_red)
        cv2.rectangle(frame, (xr, yr), (xr+wr, yr+hr), (0, 0, 255), 2)

    for contour_green in contours_green:
        xg, yg, wg, hg = cv2.boundingRect(contour_green)
        cv2.rectangle(frame, (xg, yg), (xg+wg, yg+hg), (0, 255, 0), 2)
    
    out.write(frame)

    
    if cv2.countNonZero(mask) > 0:
        print("R:1")
    else:
        print("R:0")

    
    if cv2.countNonZero(mask_green) > 0:
        print("V:1")
    else:
        print("V:0")

    
    if cv2.waitKey == ord('q'):
        break


cap.release()
out.release()
cv2.destroyAllWindows()