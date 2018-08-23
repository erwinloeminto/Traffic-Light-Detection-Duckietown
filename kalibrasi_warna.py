import numpy as np
import cv2

cap = cv2.VideoCapture(0)

def nothing(*arg):
    pass

cv2.namedWindow('RGB')
#cv2.setMouseCallback('RGB', onMouse)
# Trackbar Slider untuk mengatur nilai RGB
cv2.createTrackbar('lower - red', 'RGB', 0, 255, nothing)
cv2.createTrackbar('lower - green', 'RGB', 0, 255, nothing)
cv2.createTrackbar('lower - blue', 'RGB', 0, 255, nothing)

cv2.createTrackbar('upper - red', 'RGB', 1, 255, nothing)
cv2.createTrackbar('upper - green', 'RGB', 1, 255, nothing)
cv2.createTrackbar('upper - blue', 'RGB', 1, 255, nothing)

while True:
    # Capture frame-by-frame
    ret, image = cap.read()
	
    frame = image[150:300, 250:800]

    thrs1 = cv2.getTrackbarPos('lower - red', 'RGB')
    thrs2 = cv2.getTrackbarPos('lower - green', 'RGB')
    thrs3 = cv2.getTrackbarPos('lower - blue', 'RGB')
    thrs4 = cv2.getTrackbarPos('upper - red', 'RGB')
    thrs5 = cv2.getTrackbarPos('upper - green', 'RGB')
    thrs6 = cv2.getTrackbarPos('upper - blue', 'RGB')
	
	# Memastikan Nilai Lower lebih rendah dari Nilai Upper
    if(thrs1 > thrs4):
        cv2.setTrackbarPos('lower - red', 'RGB', thrs4 - 1)
    if(thrs2 > thrs5):
        cv2.setTrackbarPos('lower - green', 'RGB', thrs5 - 1)
    if(thrs3 > thrs6):
        cv2.setTrackbarPos('lower - blue', 'RGB', thrs6 - 1)

    # define the list of boundaries
    boundaries = [
	([thrs3, thrs2, thrs1], [thrs6, thrs5, thrs4])
    ]

    # loop over the boundaries
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        # find the colors within the specified boundaries and apply
        # the mask
		# Convert warna BGR ke HSV, menangkap range warna, kemudian proses erode dan dilate
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
		# Menggabungkan antara range warna yang ditangkap dengan gambar aslinya 
        output = cv2.bitwise_and(frame, frame, mask=mask)
		# Menampilkan kedua gambar (Output Kamera dengan Hasil Color Detection)
        imageOut = np.hstack([frame, output])

    # Display the resulting frame
	# Display pop up frame pada layar
    cv2.imshow('RGB',imageOut)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
