package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ManualObjectDetectionPipeline extends OpenCvPipeline {
  private static final String TAG = "ModPipeline";

  // Volatile because accessed by OpMode without syncObject
  public volatile boolean error = false;
  public volatile Exception lastException;

  private final Mat alternateColorMat = new Mat();
  private final Mat processedMat = new Mat();

  public final GameElement gameElement;

  public ManualObjectDetectionPipeline() {
    gameElement = new GameElement();
    gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
    gameElement.tag = "Custom";
  }

  public void DecreaseLowerHue() {
    if (gameElement.lowerColorThreshold.val[0] > 1) {
      gameElement.lowerColorThreshold.val[0]--;
    }
  }

  public void IncreaseLowerHue() {
    if (gameElement.lowerColorThreshold.val[0] < 255) {
      gameElement.lowerColorThreshold.val[0]++;
    }
  }

  public void DecreaseUpperHue() {
    if (gameElement.upperColorThreshold.val[0] > 1) {
      gameElement.upperColorThreshold.val[0]--;
    }
  }

  public void IncreaseUpperHue() {
    if (gameElement.upperColorThreshold.val[0] < 255) {
      gameElement.upperColorThreshold.val[0]++;
    }
  }

  @SuppressLint("DefaultLocale")
  @Override
  public Mat processFrame(Mat input) {
    try {
      error = false;
      lastException = null;
      gameElement.invalidate();
      // Process Image
      Imgproc.cvtColor(input, alternateColorMat, gameElement.colorConversionCode);
      Core.inRange(alternateColorMat, gameElement.lowerColorThreshold, gameElement.upperColorThreshold, processedMat);
      // Remove Noise
      Imgproc.morphologyEx(processedMat, processedMat, Imgproc.MORPH_OPEN, new Mat());
      Imgproc.morphologyEx(processedMat, processedMat, Imgproc.MORPH_CLOSE, new Mat());
      // Find Contours
      List<MatOfPoint> contours = new ArrayList<>();
      Imgproc.findContours(processedMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

      // Loop Through Contours
      for (MatOfPoint contour : contours) {
        double contourArea = Imgproc.contourArea(contour);
        if (contourArea >= gameElement.minSize.area()
            && contourArea > gameElement.area
            && contourArea <= gameElement.maxSize.area()) {
          Point[] contourArray = contour.toArray();
          MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
          RotatedRect rRect = Imgproc.minAreaRect(areaPoints);
          areaPoints.release();
          gameElement.validate(rRect, 0, contourArea);
        }

        contour.release();
      }

      if (gameElement.elementFound()) {
        Imgproc.ellipse(input, gameElement.rotatedRect, gameElement.elementColor, gameElement.borderSize);

        // Display Data
        Imgproc.putText(input, String.format("%s: %.0f (%.0f, %.0f)",
                gameElement.tag,
                gameElement.boundingRect.area(),
                FtcUtils.getMidpointX(gameElement.boundingRect),
                FtcUtils.getMidpointY(gameElement.boundingRect)),
            new Point(gameElement.boundingRect.x,
                gameElement.boundingRect.y + 20),
            Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, gameElement.elementColor, 2);
      }
    } catch (Exception exception) {
      lastException = exception;
      error = true;
    }

    return input;
  }
}
