package org.firstinspires.ftc.teamcode.qubit.core;

import java.util.ArrayList;
import java.util.List;

public class CannonControlData {
  public static double AUDIENCE_MIN_DISTANCE;
  public static double AUDIENCE_MAX_DISTANCE;
  public static double GOAL_MIN_DISTANCE;
  public static double GOAL_MAX_DISTANCE;
  public static double GOAL_SWEET_SPOT_DISTANCE;
  public static double GOAL_SWEET_SPOT_DISTANCE3;
  public static double AUDIENCE_DISTANCE;
  public static final double HOOD_MIN_POSITION = 0.3200;
  public static final double HOOD_MAX_POSITION = 0.5000;
  public static double HOOD_REGULAR_POSITION;
  public static double HOOD_MID_POSITION;
  public static double HOOD_FAR_POSITION;
  private static final List<CannonControlData> controlData;
  public final double distance;
  public final double velocity;
  public final double hoodPosition;

  static {
    controlData = getLocalizerBasedData();
  }

  public CannonControlData(double distance, double velocity, double hoodPosition) {
    this.distance = distance;
    this.velocity = velocity;
    this.hoodPosition = hoodPosition;
  }

  /**
   * Finds the closest matching power for the given distance.
   *
   * @param distance The distance to the goal april tag.
   * @return Power for the cannon motors.
   */
  public static CannonControlData getClosestData(double distance) {
    CannonControlData closestData = controlData.get(0);
    double minDifference = Math.abs(distance - closestData.distance);
    for (int i = 1; i < controlData.size(); i++) {
      CannonControlData currentData = controlData.get(i);
      double currentDifference = Math.abs(distance - currentData.distance);
      if (currentDifference < minDifference) {
        minDifference = currentDifference;
        closestData = currentData;
      }
    }

    return closestData;
  }

  public static List<CannonControlData> getLocalizerBasedData() {
    GOAL_MIN_DISTANCE = 0;
    GOAL_SWEET_SPOT_DISTANCE3 = 50;
    GOAL_SWEET_SPOT_DISTANCE = 60;
    GOAL_MAX_DISTANCE = 115;
    AUDIENCE_MIN_DISTANCE = 125;
    AUDIENCE_DISTANCE = 125;
    AUDIENCE_MAX_DISTANCE = 140;

    HOOD_REGULAR_POSITION = HOOD_MAX_POSITION;
    HOOD_MID_POSITION = 0.4500;
    HOOD_FAR_POSITION = 0.3900;

    // This is the cannon velocity data for localizer distance
    List<CannonControlData> controlData = new ArrayList<>(24);
    controlData.add(new CannonControlData(GOAL_MIN_DISTANCE, FtcCannon.CANNON_IDLE_VELOCITY, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(30, FtcCannon.CANNON_IDLE_VELOCITY, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(35, FtcCannon.CANNON_IDLE_VELOCITY, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(40, 1020, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(45, 1020, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE3, 1040, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(55, 1040, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE, 1040, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(65, 1060, HOOD_MID_POSITION));
    controlData.add(new CannonControlData(70, 1080, HOOD_MID_POSITION));
    controlData.add(new CannonControlData(75, 1100, HOOD_MID_POSITION));
    controlData.add(new CannonControlData(80, 1120, HOOD_MID_POSITION));
    controlData.add(new CannonControlData(85, 1140, 0.4450));
    controlData.add(new CannonControlData(90, 1160, 0.4450));
    controlData.add(new CannonControlData(95, 1180, 0.4400));
    controlData.add(new CannonControlData(100, 1220, 0.4350));
    controlData.add(new CannonControlData(105, 1240, 0.4350));
    controlData.add(new CannonControlData(110, 1280, 0.4270));
    controlData.add(new CannonControlData(GOAL_MAX_DISTANCE, 1300, 0.4200));
    controlData.add(new CannonControlData(120, 1340, 0.4100));
    controlData.add(new CannonControlData(AUDIENCE_DISTANCE, 1410, HOOD_FAR_POSITION));
    controlData.add(new CannonControlData(130, 1440, HOOD_FAR_POSITION));
    controlData.add(new CannonControlData(135, 1440, HOOD_FAR_POSITION));
    controlData.add(new CannonControlData(AUDIENCE_MAX_DISTANCE, 1460, HOOD_FAR_POSITION));
    return controlData;
  }
}
