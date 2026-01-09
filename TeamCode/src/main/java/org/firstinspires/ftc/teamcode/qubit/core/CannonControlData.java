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
  public static List<CannonControlData> controlData;
  public final double distance;
  public final double velocity;

  static {
    controlData = getLocalizerBasedData();
  }

  public CannonControlData(double distance, double velocity) {
    this.distance = distance;
    this.velocity = velocity;
  }

  public static List<CannonControlData> getCameraBasedData() {
    // This is the cannon velocity data for aprilTag distance
    // camera distances (supposedly in inches)

    GOAL_MIN_DISTANCE = 0;
    GOAL_SWEET_SPOT_DISTANCE = 45;
    GOAL_SWEET_SPOT_DISTANCE3 = 35;
    GOAL_MAX_DISTANCE = 75;
    AUDIENCE_MIN_DISTANCE = 100;
    AUDIENCE_DISTANCE = 115;
    AUDIENCE_MAX_DISTANCE = 120;

    List<CannonControlData> controlData = new ArrayList<>();
    controlData.add(new CannonControlData(GOAL_MIN_DISTANCE, FtcCannon.CANNON_IDLE_VELOCITY));
    controlData.add(new CannonControlData(30, 1040));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE3, 1040));
    controlData.add(new CannonControlData(40, 1060));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE, 1080));
    controlData.add(new CannonControlData(50, 1100));
    controlData.add(new CannonControlData(55, 1120));
    controlData.add(new CannonControlData(60, 1140));
    controlData.add(new CannonControlData(65, 1160));
    controlData.add(new CannonControlData(70, 1180));
    controlData.add(new CannonControlData(GOAL_MAX_DISTANCE, 1220));
    controlData.add(new CannonControlData(AUDIENCE_MIN_DISTANCE, 1400));
    controlData.add(new CannonControlData(105, 1400));
    controlData.add(new CannonControlData(110, 1420));
    controlData.add(new CannonControlData(AUDIENCE_DISTANCE, 1420));
    controlData.add(new CannonControlData(AUDIENCE_MAX_DISTANCE, 1440));
    return controlData;
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

    // This is the cannon velocity data for localizer distance
    List<CannonControlData> controlData = new ArrayList<>(24);
    controlData.add(new CannonControlData(GOAL_MIN_DISTANCE, FtcCannon.CANNON_IDLE_VELOCITY));
    controlData.add(new CannonControlData(30, FtcCannon.CANNON_IDLE_VELOCITY));
    controlData.add(new CannonControlData(35, FtcCannon.CANNON_IDLE_VELOCITY));
    controlData.add(new CannonControlData(40, 1020));
    controlData.add(new CannonControlData(45, 1020));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE3, 1040));
    controlData.add(new CannonControlData(55, 1040));
    controlData.add(new CannonControlData(GOAL_SWEET_SPOT_DISTANCE, 1040));
    controlData.add(new CannonControlData(65, 1060));
    controlData.add(new CannonControlData(70, 1080));
    controlData.add(new CannonControlData(75, 1100));
    controlData.add(new CannonControlData(80, 1120));
    controlData.add(new CannonControlData(85, 1140));
    controlData.add(new CannonControlData(90, 1160));
    controlData.add(new CannonControlData(95, 1180));
    controlData.add(new CannonControlData(100, 1200));
    controlData.add(new CannonControlData(105, 1220));
    controlData.add(new CannonControlData(110, 1260));
    controlData.add(new CannonControlData(GOAL_MAX_DISTANCE, 1280));
    controlData.add(new CannonControlData(120, 1320));
    controlData.add(new CannonControlData(AUDIENCE_DISTANCE, 1400));
    controlData.add(new CannonControlData(130, 1400));
    controlData.add(new CannonControlData(135, 1460));
    controlData.add(new CannonControlData(AUDIENCE_MAX_DISTANCE, 1480));
    return controlData;
  }
}
