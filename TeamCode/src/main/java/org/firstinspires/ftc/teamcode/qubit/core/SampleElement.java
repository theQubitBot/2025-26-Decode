package org.firstinspires.ftc.teamcode.qubit.core;

import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObjectColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.GeometricShapeEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObjectDetectionAlgorithmEnum;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * A class to manage the Sample
 */
public class SampleElement {
  private static final String TAG = "SampleElement";
  public static final int ATTENDANCE_MAX = 30;
  public static final int ATTENDANCE_THRESHOLD = 15;
  public double minAspectRatio, maxAspectRatio;
  public double area;
  public double aspectRatio;
  public Scalar elementColor;
  public Scalar lowerColorThreshold;
  public Scalar upperColorThreshold;
  public Rect minSize, maxSize;
  public final int borderSize;
  public int colorConversionCode;
  public String tag;
  public GeometricShapeEnum geometricShapeEnum;
  public boolean found;
  public Rect boundingRect;
  public RotatedRect rotatedRect;
  private final int[] attendanceRegister;
  private int attendanceIndex;

  public ObjectDetectionAlgorithmEnum tgeDetectionAlgorithm;

  /**
   * Constructor. Sets the defaults for the TGE.
   */
  public SampleElement() {
    // OpenCV processes about 6 FPS
    // Look back 5 seconds of processed frames
    attendanceRegister = new int[ATTENDANCE_MAX];
    attendanceIndex = 0;
    aspectRatio = 0;
    boundingRect = new Rect();
    rotatedRect = new RotatedRect();
    borderSize = 4;
    colorConversionCode = Imgproc.COLOR_RGB2HSV;
    geometricShapeEnum = GeometricShapeEnum.UNKNOWN;
    elementColor = FtcColorUtils.RGB_WHITE;
    tgeDetectionAlgorithm = ObjectDetectionAlgorithmEnum.CONTOUR;
    found = false;
    lowerColorThreshold = new Scalar(0.0, 0.0, 0.0);
    upperColorThreshold = new Scalar(255.0, 255.0, 255.0);
    tag = FtcColorUtils.TAG_UNKNOWN;
    minSize = new Rect(0, 0, 70, 70);
    maxSize = new Rect(0, 0, 300, 300);

    // Team prop merges with spike mark, aspect ratio can be 3!
    minAspectRatio = 0.25;
    maxAspectRatio = 4.00;
  }

  public int attendanceCount() {
    int count = 0;
    synchronized (attendanceRegister) {
      for (int i = 0; i < attendanceRegister.length; i++) {
        count += attendanceRegister[i];
      }
    }

    return count;
  }

  public boolean elementConsistentlyPresent() {
    return attendanceCount() >= ATTENDANCE_THRESHOLD;
  }

  public synchronized boolean elementFound() {
    return found;
  }

  public static SampleElement getSample(ObjectColorEnum gameElementType) {
    SampleElement gameElement = new SampleElement();
    switch (gameElementType) {
      case BLUE:
        gameElement.elementColor = FtcColorUtils.RGB_BLUE;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(110.0, 20.0, 50.0);
        gameElement.upperColorThreshold = new Scalar(130.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_BLUE;
        break;
      case RED1:
        gameElement.elementColor = FtcColorUtils.RGB_RED;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(0.0, 20.0, 50.0);
        gameElement.upperColorThreshold = new Scalar(15.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_RED1;
        break;
      case RED2:
        gameElement.elementColor = FtcColorUtils.RGB_RED;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(165.0, 20.0, 50.0);
        gameElement.upperColorThreshold = new Scalar(180.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_RED2;
        break;
      case YELLOW:
        // Yellow Junctions
        gameElement.elementColor = FtcColorUtils.RGB_YELLOW;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(20.0, 100.0, 100.0);
        gameElement.upperColorThreshold = new Scalar(40.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_YELLOW;
        break;
    }

    return gameElement;
  }

  public synchronized Point getMidPoint() {
    return FtcUtils.getMidpoint(boundingRect);
  }

  public synchronized void invalidate() {
    area = 0;
    boundingRect = new Rect();
    rotatedRect = new RotatedRect();
    found = false;
  }

  private void markAbsent() {
    synchronized (attendanceRegister) {
      attendanceRegister[attendanceIndex] = 0;
      attendanceIndex = (attendanceIndex + 1) % attendanceRegister.length;
    }
  }

  private void markPresent() {
    synchronized (attendanceRegister) {
      attendanceRegister[attendanceIndex] = 1;
      attendanceIndex = (attendanceIndex + 1) % attendanceRegister.length;
    }
  }

  public void takeAttendance() {
    if (elementFound()) {
      markPresent();
    } else {
      markAbsent();
    }
  }

  public synchronized void validate(RotatedRect rRect, double aspectRatio, double area) {
    FtcUtils.copyRect(rRect, rotatedRect);
    FtcUtils.copyRect(rotatedRect.boundingRect(), boundingRect);
    this.area = area;
    this.aspectRatio = aspectRatio;
    found = true;
  }
}
