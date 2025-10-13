package org.firstinspires.ftc.teamcode.qubit.core;

import org.firstinspires.ftc.teamcode.qubit.core.enumerations.GeometricShapeEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObjectColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObjectDetectionAlgorithmEnum;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * A class to manage the Team Game Element (TGE).
 */
public class GameElement {
  private static final String TAG = "GameElement";
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
  public GameElement() {
    // OpenCV processes about 6 FPS
    // Look back 5 seconds of processed frames
    attendanceRegister = new int[ATTENDANCE_MAX];
    attendanceIndex = 0;
    aspectRatio = 0;
    boundingRect = new Rect();
    rotatedRect = new RotatedRect();
    borderSize = 4;
    colorConversionCode = Imgproc.COLOR_RGB2YCrCb;
    geometricShapeEnum = GeometricShapeEnum.UNKNOWN;
    elementColor = FtcColorUtils.RGB_WHITE;
    tgeDetectionAlgorithm = ObjectDetectionAlgorithmEnum.CONTOUR_AND_CHANNEL;
    found = false;
    lowerColorThreshold = new Scalar(0.0, 0.0, 0.0);
    upperColorThreshold = new Scalar(255.0, 255.0, 255.0);
    tag = FtcColorUtils.TAG_UNKNOWN;
    minSize = new Rect(0, 0, 80, 80);
    maxSize = new Rect(0, 0, 400, 400);

    // Team prop merges with spike mark, aspect ratio can be 3!
    minAspectRatio = 0.25;
    maxAspectRatio = 3.5;
  }

  public int attendanceCount() {
    int sum = 0;
    synchronized (attendanceRegister) {
      for (int j : attendanceRegister) {
        sum += j;
      }
    }

    return sum;
  }

  public boolean elementConsistentlyPresent() {
    return attendanceCount() >= ATTENDANCE_THRESHOLD;
  }

  public synchronized boolean elementFound() {
    return found;
  }

  public static GameElement getGE(ObjectColorEnum gameElementType) {
    GameElement gameElement = new GameElement();
    switch (gameElementType) {
      case BLUE:
        gameElement.elementColor = FtcColorUtils.RGB_BLUE;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(110.0, 20.0, 50.0);
        gameElement.upperColorThreshold = new Scalar(130.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_BLUE;
        break;
      case BLACK:
        gameElement.elementColor = FtcColorUtils.RGB_BLACK;
        gameElement.lowerColorThreshold = new Scalar(0.0, 63.0, 63.0);
        gameElement.upperColorThreshold = new Scalar(31.0, 191.0, 191.0);
        gameElement.tag = FtcColorUtils.TAG_BLACK;
        break;
      case GRAY:
        gameElement.elementColor = FtcColorUtils.RGB_GRAY;
        gameElement.lowerColorThreshold = new Scalar(63.0, 63.0, 63.0);
        gameElement.upperColorThreshold = new Scalar(191.0, 191.0, 191.0);
        gameElement.tag = FtcColorUtils.TAG_GRAY;
        break;
      case GREEN:
        gameElement.elementColor = FtcColorUtils.RGB_GREEN;
                /* YCbCr
                gameElement.lowerColorThreshold = new Scalar(63.0, 0.0, 0.0);
                gameElement.upperColorThreshold = new Scalar(255.0, 120.0, 120.0);
                */

        //* HSV
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(45.0, 20.0, 50.0);
        gameElement.upperColorThreshold = new Scalar(86.0, 255.0, 255.0);
        //*/
        gameElement.tag = FtcColorUtils.TAG_GREEN;
        break;
      case ORANGE:
        gameElement.elementColor = FtcColorUtils.RGB_ORANGE;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(11.0, 10.0, 10.0);
        gameElement.upperColorThreshold = new Scalar(24.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_ORANGE;
        break;
      case PINK:
        gameElement.elementColor = FtcColorUtils.RGB_PINK;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(150.0, 10.0, 10.0);
        gameElement.upperColorThreshold = new Scalar(170.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_PINK;
        break;
      case PURPLE:
        gameElement.elementColor = FtcColorUtils.RGB_PURPLE;
        gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
        gameElement.lowerColorThreshold = new Scalar(140.0, 10.0, 10.0);
        gameElement.upperColorThreshold = new Scalar(160.0, 255.0, 255.0);
        gameElement.tag = FtcColorUtils.TAG_PURPLE;
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
      case WHITE:
        gameElement.elementColor = FtcColorUtils.RGB_WHITE;
        gameElement.lowerColorThreshold = new Scalar(191.0, 63.0, 63.0);
        gameElement.upperColorThreshold = new Scalar(255.0, 191.0, 191.0);
        gameElement.tag = FtcColorUtils.TAG_WHITE;
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
