import cv2
#from pyzbar import pyzbar

img = cv2.imread('/home/woobin/Pictures/qr_code.png')

cv2.imshow("img",img)
cv2.waitKey(0)
cv2.destroyAllWindows()