# 640x480 90fps, exposure 1500, gain 20, red balance 1300, blue balance 1500
# green - 55-95, 145-255, 50-255

# contour filtering
# largest, area 20-100, fullness 48-100, W/H 0.5-2.0, speckle 40
# erosion = 0, dilation = 5x5 once

import cv2
import numpy as np

line = int(1)

# --- Define color ranges in HSV ---
COLOR_RANGES = {
    "green": ((55, 145, 50), (95, 255, 255)),
    "purple": ((145, 0, 0), (165, 255, 255)),
}

COLOR_THRESHOLDS = {
    "green": 10,
    "purple": 5
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
    threshhold = 0
    highestPercent = 0
    largestContour = np.array([[]])
    x, y, w, h = 0, 0, 0, 0
    llPython = [-1, x, y, w, h, 0, 0, 0]

    global line
    line = 1
    drawText(frame, "-----------")

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # *** OpenCV Dilation Operation ***
    # Define a structuring element (kernel) for dilation
    # A 5x5 rectangular kernel is a common choice
    kernel = np.ones((5, 5), np.uint8)

    for colorName in list(COLOR_RANGES.keys()):
        lowerRange, upperRange = COLOR_RANGES.get(colorName)
        mask = cv2.inRange(hsv, lowerRange, upperRange)

        # Apply the dilation operation
        # 'iterations' determines how many times the dilation is applied
        dilated_image = cv2.dilate(mask, kernel, iterations=1)
        matchPercent = 100 * (cv2.countNonZero(dilated_image) / dilated_image.size)
        threshhold = COLOR_THRESHOLDS[colorName]
        if matchPercent >= threshhold and matchPercent > highestPercent:
            highestPercent = matchPercent
            dominantColor = colorName
            llPython[0] = lowerRange[0]
            llPython[5] = matchPercent
            contours, _ = cv2.findContours(dilated_image, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                # cv2.drawContours(frame, contours, -1, 255, 2)
                largestContour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largestContour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

    drawText(frame, f"Dominant color: {dominantColor}, hue: {llPython[0]} {llPython[5]:.0f}%")
    return largestContour, frame, llPython
