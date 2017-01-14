
import cv2
import matplotlib.pyplot as py

# Start the camera
cv2.VideoCapture(0)[D[D[D[D[D[D[D[D[D[D[D[D[D[D[D[D[D[D[Dcap = [2~cv2.c[2Videocapture(0)
[D[D[D[D[D[D[D[D[D[D[A[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[C[D[D[D[D[D[1;2D[1;2CC[Ca[C[C[C[C[C[C[C[C[C[C[D[D

while True:
[D[A[C[C[C[C[C[C([C[C[D[DTrue):
   ret,frame = cap.read()
   cv2.imshow('myfig',frame)
   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

cap.release()
cv2.destroyAllWindows()
