import cv2

# Initialize the camera capture
capture = cv2.VideoCapture(0)  # 0 means default camera

# Check if the camera is opened successfully
if not capture.isOpened():
    print("Error: Couldn't open the camera.")
    exit()

# Capture a frame
ret, frame = capture.read()

# Check if the frame was captured successfully
if not ret:
    print("Error: Couldn't capture frame.")
    exit()

# Save the captured frame to a file
cv2.imwrite('captured_image.jpg', frame)

# Release the camera capture
capture.release()
