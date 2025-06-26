package org.firstinspires.ftc.teamcode.qubit.core;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ObjectDetectionBase {
  protected final Rect leftRect, centerRect, rightRect;
  protected final ArrayList<Rect> targetMatRects;

  public ObjectDetectionBase() {
    leftRect = new Rect(new Point(0, 0), new Point(0, 0));
    centerRect = new Rect(new Point(50, 230), new Point(370, 370));
    rightRect = new Rect(new Point(380, 230), new Point(640, 420));
    targetMatRects = new ArrayList<>(2);
    targetMatRects.add(centerRect);
    targetMatRects.add(rightRect);
  }

  /**
   * Blots out region of the frame that is outside the target areas.
   *
   * @param frame The input image frame.
   */
  protected void blotFrame(Mat frame) {
    // Save original frame.
    Mat cloneSubMat, frameSubMat;
    Mat clonedMat = frame.clone();

    // Blot out the entire original frame.
    Imgproc.rectangle(frame, FtcOpenCvCam.cameraRect, FtcColorUtils.RGB_WHITE, Imgproc.FILLED);

    // Copy the desired areas from clone to original frame.
    for (Rect rect : targetMatRects) {
      cloneSubMat = clonedMat.submat(rect);
      frameSubMat = frame.submat(rect);
      cloneSubMat.copyTo(frameSubMat);
      cloneSubMat.release();
      frameSubMat.release();
    }

    clonedMat.release();
  }
}
