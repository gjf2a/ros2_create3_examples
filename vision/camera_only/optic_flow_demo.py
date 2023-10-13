# Adapted from:
#   https://www.perplexity.ai/search/801bd5b6-6ba1-4816-8308-00952e897ef7?s=u


import time

class Timer:
    def __init__(self):
        self.start = time.time()
        self.count = 0

    def inc(self):
        self.count += 1

    def elapsed(self):
        return self.count / (time.time() - self.start)


import cv2
import numpy as np

# Load the video
cap = cv2.VideoCapture(0)

# Read the first frame
ret, frame = cap.read()

frame = cv2.resize(frame, (320, 240))

# Convert the frame to grayscale
prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

timer = Timer()

while True:
    # Create a mask for the ground
    mask = np.zeros_like(frame)

    # Read the next frame
    ret, frame = cap.read()

    frame = cv2.resize(frame, (320, 240))

    if not ret:
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate the sparse optical flow
    flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

    # Compute the magnitude and angle of the optical flow
    magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])

    # Set the mask to 1 for pixels with a small angle and a large magnitude
    mask[(angle < np.pi / 4) & (magnitude > 2)] = 255

    # Display the resulting mask
    cv2.imshow('Mask', mask)
    timer.inc()

    # Update the previous frame
    prev_gray = gray

    # Check if the 'q' key was pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the windows
print("FPS:", timer.elapsed())
cap.release()
cv2.destroyAllWindows()
