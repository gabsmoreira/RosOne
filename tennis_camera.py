import numpy as np
import cv2
import cv2.cv as cv

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX
range_R = np.arange(40,205)
range_G = np.arange(70,255)
range_B = np.arange(0,60)

lower_green = np.array([20,70,100])
upper_green = np.array([55,255,255])
while True:
	ret, img = cap.read()
	blur =cv2.GaussianBlur(img,(5,5),10)
	gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_green, upper_green)
	mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
	edges = cv2.Canny(mask,100,200)

	circles = cv2.HoughCircles(edges, cv.CV_HOUGH_GRADIENT, 2, 1000, param1 = 100,param2=40)


	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for (x, y, r) in circles:
			cv2.circle(img, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
			if mask_rgb[y][x][0] == 255:
				if mask_rgb[y][x][1] == 255: 
					if mask_rgb[y][x][2] == 255:
						cv2.circle(img, (x, y), r, (0, 255, 0), 4)
						cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)


	cv2.imshow("imagem", img)
	cv2.imshow("output",edges)

	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break
cap.release()
cv2.destroyAllWindows()