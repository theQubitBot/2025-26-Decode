import cv2
import numpy as np

# --- User Configuration ---
TARGET_COLOR = "red1"  # Choose from: "red", "green", "blue", "yellow", etc.
THRESHOLD_PERCENT = int(1)  # % of image pixels that must match the color to count as "predominant"
line = int(1)

# --- Define color ranges in HSV ---
COLOR_RANGES = {
    "red1": ((0, 50, 70), (10, 255, 255)),
    "red2": ((170, 50, 70), (180, 255, 255)),
    "artifact_green": ((55, 50, 70), (85, 255, 255)),
    "green": ((55, 50, 70), (85, 255, 255)),
    "blue": ((90, 50, 70), (130, 255, 255)),
    "artifact_purple": ((150, 50, 70), (165, 255, 255)),
    "purple": ((150, 50, 70), (165, 255, 255)),
    "yellow": ((20, 50, 50), (30, 255, 255))
}


def drawText(frame, message):
    global line
    y = 20 * line
    cv2.putText(frame,
                message,
                (0, y), cv2.FONT_HERSHEY_SIMPLEX,
                .5, (0, 255, 0), 1, cv2.LINE_AA)
    line += 1
    print(message)


def runPipeline(frame, llrobot):
    global line
    line = 1
    drawText(frame, "-----------")
    frame_height, frame_width, channels = frame.shape
    drawText(frame, f"Frame size: {frame_width}x{frame_height}")

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask for target color
    lower, upper = COLOR_RANGES.get(TARGET_COLOR)
    lower_np = np.array(lower, dtype=np.uint8)
    upper_np = np.array(upper, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_np, upper_np)

    # Calculate percentage of pixels that match the target color
    match_percent = 100 * (cv2.countNonZero(mask) / mask.size)
    drawText(frame, f"Mask: {cv2.countNonZero(mask)} / {mask.size} = {match_percent:.1f}")

    # find contours in the new binary frame
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largestContour = np.array([[]])
    x, y, w, h = 0, 0, 0, 0
    if len(contours) > 0:
        cv2.drawContours(frame, contours, -1, 255, 2)

        # record the largest contour
        largestContour = max(contours, key=cv2.contourArea)

        # get the unrotated bounding box that surrounds the contour
        x, y, w, h = cv2.boundingRect(largestContour)

        drawText(frame, f"Largest contour area: {w * h}")

        # draw the unrotated bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

    # Print detection result
    if match_percent >= THRESHOLD_PERCENT:
        drawText(frame, f"{TARGET_COLOR}: yes")
        llpython = [1, x, y, w, h, 0, 0, 0]
    else:
        drawText(frame, f"{TARGET_COLOR}: no")
        llpython = [0, x, y, w, h, 0, 0, 0]

    return largestContour, frame, llpython
