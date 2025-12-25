package org.firstinspires.ftc.teamcode.qubit.core;

public class CannonControlData {
  // camera distances (supposedly in inches)
  public static final double C0 = 0;
  public static final double C30 = 30;
  public static final double C35 = 35;
  public static final double C40 = 40;
  public static final double C45 = 45;
  public static final double C50 = 50;
  public static final double C55 = 55;
  public static final double C60 = 60;
  public static final double C65 = 65;
  public static final double C70 = 70;
  public static final double C75 = 75;
  public static final double C100 = 100;
  public static final double C105 = 105;
  public static final double C110 = 110;
  public static final double C115 = 115;
  public static final double C120 = 120;

  // localizer distance (inches)
  public static final double L0 = 0;
  public static final double L30 = 32.1;
  public static final double L35 = 37.8;
  public static final double L40 = 43.8;
  public static final double L45 = 50.0;
  public static final double L50 = 55.9;
  public static final double L55 = 61.8;
  public static final double L60 = 67.9;
  public static final double L65 = 73.7;
  public static final double L70 = 79.6;
  public static final double L75 = 85.4;
  public static final double L100 = 114.2;
  public static final double L105 = 120.1;
  public static final double L110 = 126.1;
  public static final double L115 = 132.1;
  public static final double L120 = 138.7;
  private final double cameraDistance;
  private final double localizerDistance;
  public final double velocity;

  public CannonControlData(double cDistance, double lDistance, double velocity) {
    this.cameraDistance = cDistance;
    this.localizerDistance = lDistance;
    this.velocity = velocity;
  }

  public double getDistance() {
    return localizerDistance;
  }
}
