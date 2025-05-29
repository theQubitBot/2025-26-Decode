package org.firstinspires.ftc.teamcode.qubit.core;

import org.opencv.core.Scalar;

/**
 * Global utility functions
 */
public final class FtcColorUtils {
  private static final String TAG = "FtcColorUtils";
  public static final double Y_MIN = 0.0;
  public static final double Y_MAX = 255.0;
  public static final double CR_MIN = 0.0;
  public static final double CR_MAX = 255.0;
  public static final double CB_MIN = 0.0;
  public static final double CB_MAX = 255.0;
  public static final int Y_CHANNEL = 0, CR_CHANNEL = 1, CB_CHANNEL = 2;

  public static final Scalar RGB_BLACK = new Scalar(0, 0, 0);
  public static final Scalar RGB_BLUE = new Scalar(0, 0, 255);
  public static final Scalar RGB_GRAY = new Scalar(127, 127, 127);
  public static final Scalar RGB_GREEN = new Scalar(0, 255, 0);
  public static final Scalar RGB_ORANGE = new Scalar(255, 165, 0);
  public static final Scalar RGB_PINK = new Scalar(196, 23, 112);
  public static final Scalar RGB_PURPLE = new Scalar(255, 0, 255);
  public static final Scalar RGB_RED = new Scalar(255, 0, 0);
  public static final Scalar RGB_TEAL = new Scalar(3, 148, 252);
  public static final Scalar RGB_YELLOW = new Scalar(255, 255, 0);
  public static final Scalar RGB_WHITE = new Scalar(255, 255, 255);

  public static final String TAG_BLACK = "Black";
  public static final String TAG_BLUE = "Blue";
  public static final String TAG_GRAY = "Gray";
  public static final String TAG_GREEN = "Green";
  public static final String TAG_ORANGE = "Orange";
  public static final String TAG_PINK = "Pink";
  public static final String TAG_PURPLE = "Purple";
  public static final String TAG_RED1 = "Red1";
  public static final String TAG_RED2 = "Red2";
  public static final String TAG_TEAL = "Teal";
  public static final String TAG_UNKNOWN = "Unknown";
  public static final String TAG_YELLOW = "Yellow";
  public static final String TAG_WHITE = "White";
}
