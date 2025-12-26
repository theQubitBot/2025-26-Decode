package org.firstinspires.ftc.teamcode.qubit.core.Field;

import com.pedropathing.geometry.Pose;

/**
 * Field origin is at field center, audience view.
 * x increases up, y increases left (Ftc coordinate system), angle increases anti-clockwise.
 * Robot center is the cannon center.
 * Robot center is over field center for all measurements below.
 */
public class FtcField {
  public static final double RADIAN0 = 0;
  public static final double RADIAN15 = Math.toRadians(15);
  public static final double RADIAN20 = Math.toRadians(20);
  public static final double RADIAN22 = Math.toRadians(22);
  public static final double RADIAN30 = Math.toRadians(30);
  public static final double RADIAN45 = Math.toRadians(45);
  public static final double RADIAN60 = Math.toRadians(60);
  public static final double RADIAN67 = Math.toRadians(67);
  public static final double RADIAN70 = Math.toRadians(70);
  public static final double RADIAN75 = Math.toRadians(75);
  public static final double RADIAN90 = Math.toRadians(90);
  public static final double RADIAN105 = Math.toRadians(105);
  public static final double RADIAN120 = Math.toRadians(120);
  public static final double RADIAN133 = Math.toRadians(133);
  public static final double RADIAN135 = Math.toRadians(135);
  public static final double RADIAN150 = Math.toRadians(150);
  public static final double RADIAN180 = Math.toRadians(179.9999); // Force robot to turn anti clockwise (looking down from above)
  public static final Pose origin = new Pose(0, 0, RADIAN0);
  public static final Pose blueGoalPose = new Pose(62.5, 62.5, RADIAN45);
  public static final Pose blueGoalStartPose = new Pose(52, 49, RADIAN45);
  public static final Pose blueAudienceStartPose = new Pose(-62.5, 16, RADIAN0);
  public static final Pose blueParkingPose = new Pose(-36.5, -35.5, RADIAN90);
  public static final Pose redGoalPose = new Pose(62.5, -62.5, -RADIAN45);
  public static final Pose redGoalStartPose = new Pose(52, -49, -RADIAN45);
  public static final Pose redAudienceStartPose = new Pose(-62.5, -16, RADIAN0);
  public static final Pose redParkingPose = new Pose(-36.5, 35.5, -RADIAN90);

  public static final Triangle goalLaunchTriangle = new Triangle(
      new Pose(62, 62, RADIAN0),
      new Pose(0, 0, RADIAN0),
      new Pose(62, -62, RADIAN0));
  public static final Triangle audienceLaunchTriangle = new Triangle(
      new Pose(-62, 23.5, RADIAN0),
      new Pose(-43, 0, RADIAN0),
      new Pose(-62, -23.5, RADIAN0));
}
