# 640x480 90fps, exposure 1500, gain 20, red balance 1300, blue balance 1500
# green - 55-95, 145-255, 50-255

# contour filtering
# largest, area 20-100, fullness 48-100, W/H 0.5-2.0, speckle 40

import cv2
import numpy as np

THRESHOLD_PERCENT = int(10)  # % of image pixels that must match the color to count as "predominant"
line = int(1)

# --- Define color ranges in HSV ---
COLOR_RANGES = {
    "red1": ((0, 0, 0), (20, 125, 125)),
    "orange": ((11, 20, 50), (24, 255, 255)),
    "yellow": ((25, 20, 50), (44, 255, 255)),
    "green": ((55, 140, 50), (95, 255, 255)),
    "blue": ((110, 20, 50), (130, 255, 255)),
    "purple": ((143, 20, 50), (163, 255, 255)),
    "pink": ((164, 20, 50), (169, 255, 255)),
    "red2": ((170, 20, 50), (180, 255, 255))
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
    dominantColor = "none"
    highestPercent = int(0)
    largestContour = np.array([[]])
    x, y, w, h = 0, 0, 0, 0
    llPython = [-1, x, y, w, h, 0, 0, 0]

    global line
    line = 1
    drawText(frame, "-----------")

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for colorName in list(COLOR_RANGES.keys()):
        lowerRange, upperRange = COLOR_RANGES.get(colorName)
        lower_np = np.array(lowerRange, dtype=np.uint8)
        upper_np = np.array(upperRange, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_np, upper_np)
        matchPercent = 100 * (cv2.countNonZero(mask) / mask.size)
        if matchPercent > THRESHOLD_PERCENT and matchPercent > highestPercent:
            highestPercent = matchPercent
            dominantColor = colorName

            # hue
            llPython[0] = lowerRange[0]

            # percent
            llPython[5] = highestPercent
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                # cv2.drawContours(frame, contours, -1, 255, 2)
                largestContour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largestContour)

    # draw largest contour
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

    drawText(frame, f"Dominant color: {dominantColor}, hue: {llPython[0]} {llPython[5]:.0f}%")
    return largestContour, frame, llPython
