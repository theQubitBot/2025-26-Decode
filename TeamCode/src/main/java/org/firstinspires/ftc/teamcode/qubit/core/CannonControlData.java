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
  private static double VELOCITY_BUMP;
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

    VELOCITY_BUMP = 10;

    HOOD_REGULAR_POSITION = HOOD_MAX_POSITION;
    HOOD_MID_POSITION = 0.4500;
    HOOD_FAR_POSITION = 0.3900;

    // This is the cannon velocity data for localizer distance
    List<CannonControlData> controlData = new ArrayList<>(24);
    controlData.add(new CannonControlData(GOAL_MIN_DISTANCE, FtcCannon.CANNON_IDLE_VELOCITY, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(30, FtcCannon.CANNON_IDLE_VELOCITY, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(35, FtcCannon.CANNON_IDLE_VELOCITY, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(40, 1020 + VELOCITY_BUMP, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(45, 1020 + VELOCITY_BUMP, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE3, 1040 + VELOCITY_BUMP, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(55, 1040 + VELOCITY_BUMP, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE, 1040 + VELOCITY_BUMP, HOOD_REGULAR_POSITION));
    controlData.add(new CannonControlData(65, 1070 + VELOCITY_BUMP, 0.4915));
    controlData.add(new CannonControlData(70, 1100 + VELOCITY_BUMP, 0.4831));
    controlData.add(new CannonControlData(75, 1120 + VELOCITY_BUMP, 0.4746));
    controlData.add(new CannonControlData(80, 1150 + VELOCITY_BUMP, 0.4662));
    controlData.add(new CannonControlData(85, 1180 + VELOCITY_BUMP, 0.4577));
    controlData.add(new CannonControlData(90, 1210 + VELOCITY_BUMP, 0.4492));
    controlData.add(new CannonControlData(95, 1240 + VELOCITY_BUMP, 0.4408));
    controlData.add(new CannonControlData(100, 1270 + VELOCITY_BUMP, 0.4323));
    controlData.add(new CannonControlData(105, 1300 + VELOCITY_BUMP, 0.4238));
    controlData.add(new CannonControlData(110, 1280 + VELOCITY_BUMP, 0.4154));
    controlData.add(new CannonControlData(GOAL_MAX_DISTANCE, 1320 + VELOCITY_BUMP, 0.4069));
    controlData.add(new CannonControlData(120, 1380 + VELOCITY_BUMP, 0.3985));
    controlData.add(new CannonControlData(AUDIENCE_DISTANCE, 1410 + VELOCITY_BUMP, HOOD_FAR_POSITION));
    controlData.add(new CannonControlData(130, 1440 + VELOCITY_BUMP, HOOD_FAR_POSITION));
    controlData.add(new CannonControlData(135, 1460 + VELOCITY_BUMP, HOOD_FAR_POSITION));
    controlData.add(new CannonControlData(AUDIENCE_MAX_DISTANCE, 1480 + VELOCITY_BUMP, HOOD_FAR_POSITION));
    return controlData;
  }
}
