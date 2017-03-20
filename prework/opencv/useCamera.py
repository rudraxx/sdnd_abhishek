# Testing video capture with opencv2.


import cv2

cap = cv2.VideoCapture(0)

while True :
    ret,frame = cap.read()
    grayframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    # Show this image
    cv2.imshow('fig',frame)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()