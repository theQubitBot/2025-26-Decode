package org.firstinspires.ftc.teamcode.qubit.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.Field.FtcField;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;

/**
 * A class to manage robot operations in Tele Op using Pedro Pathing
 */
public class TeleOpLocalizer extends FtcSubSystemBase {
  private static final String TAG = "TeleOpLocalizer";
  private Follower follower;
  private Pose startingPose;
  private Pose parkingPose;
  private Pose tagPose;

  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final BaseBot parent;

  /* Constructor */
  public TeleOpLocalizer(BaseBot robot) {
    parent = robot;
  }

  /**
   * Returns distance to the goal based on alliance color and
   * robot starting position (Goal vs Audience side).
   *
   * @return Distance to the goal (in inches).
   */
  public double getGoalDistance() {
    follower.updatePose();
    Pose robotPose = follower.getPose();
    return robotPose.distanceFrom(tagPose);
  }

  /**
   * Returns goal heading relative to robot pose.
   *
   * @return Goal heading in radians.
   */
  public double getGoalHeading() {
    follower.updatePose();
    Pose robotPose = follower.getPose();
    return Math.atan2(tagPose.getY() - robotPose.getY(), tagPose.getX() - robotPose.getX());
  }

  /**
   * Returns robot pose.
   *
   * @return Robot pose.
   */
  public Pose getRobotPose(){
    follower.updatePose();
    return follower.getPose();
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;

    // Initializes poses for robot starting position
    if (parent != null && parent.config != null) {
      startingPose = new Pose(parent.config.x, parent.config.y, parent.config.heading);

      // Regardless of robot starting position, parking and goal are fixed.
      if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
        parkingPose = FtcField.blueParkingPose;
        tagPose = FtcField.blueGoalPose;
      } else {
        parkingPose = FtcField.redParkingPose;
        tagPose = FtcField.redGoalPose;
      }
    } else {
      startingPose = new Pose(0, 0, 0);
      parkingPose = new Pose(0, 0, 0);
      tagPose = new Pose(0, 0, 0);
    }

    follower = Constants.createFollower(hardwareMap);
    follower.setMaxPower(1.0); // override any autoOp set max power.
    setStartingPose(startingPose);
    FtcLogger.exit();
  }

  /**
   * Operates the localizer using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  @Override
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();
    if (gamePad1.dpadUpWasPressed() || gamePad2.dpadUpWasPressed()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      follower.turnTo(getGoalHeading());
    } else if (gamePad1.dpad_up || gamePad2.dpad_up) {
      follower.update();
    } else if (gamePad1.dpadUpWasReleased() || gamePad2.dpadUpWasReleased()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    } else if (gamePad1.dpadDownWasPressed() || gamePad2.dpadDownWasPressed()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      follower.updatePose();
      follower.holdPoint(follower.getPose());
    } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
      follower.update();
    } else if (gamePad1.dpadDownWasReleased() || gamePad2.dpadDownWasReleased()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    //  else Nothing to do. Pose will be  updated when needed.

    if (gamePad1.shareWasPressed() || gamePad2.shareWasPressed()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      follower.updatePose();
      Pose startPose = follower.getPose();
      PathChain parkPath = follower.pathBuilder()
          .addPath(new BezierLine(startPose, parkingPose))
          .setLinearHeadingInterpolation(startPose.getHeading(), parkingPose.getHeading())
          .build();
      follower.followPath(parkPath, true);
    } else if (gamePad1.share || gamePad2.share) {
      follower.update();
    } else if (gamePad1.shareWasReleased() || gamePad2.shareWasReleased()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    FtcLogger.exit();
  }

  public void setStartingPose(Pose pose) {
    startingPose = pose;
    follower.setStartingPose(startingPose);
    follower.update();
  }

  /*
   * Displays telemetry. Helps with debugging.
   */
  @Override
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled && telemetry != null) {
      Pose robotPose = getRobotPose();
      telemetry.addData("robotPose", "%.1f %.1f %.1f",
          robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
      telemetry.addData("Goal distance", "%.1f", getGoalDistance());
      telemetry.addData("Goal heading", "%.1f", Math.toDegrees(getGoalHeading()));
      telemetry.addData("Inside launch", "%b, pointingAtGoal %b",
          robotInLaunchZone(), robotPointingAtGoal());
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    FtcLogger.exit();
  }

  /**
   * Stops the localizer.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    if (follower != null) {
      follower.breakFollowing();
      follower.updatePose();
      Pose robotPose = follower.getPose();
      parent.config.x = robotPose.getX();
      parent.config.y = robotPose.getY();
      parent.config.heading = robotPose.getHeading();
      parent.config.saveToFile();
    }

    FtcLogger.exit();
  }

  public boolean robotInLaunchZone() {
    follower.updatePose();
    Pose robotPose = follower.getPose();
    return FtcField.goalLaunchTriangle.contains(robotPose) ||
        FtcField.audienceLaunchTriangle.contains(robotPose);
  }

  public boolean robotPointingAtGoal() {
    double delta = Math.abs(getRobotPose().getHeading() - getGoalHeading());
    delta = MathFunctions.normalizeAngle(delta);
    return FtcUtils.outsideRange(delta, FtcField.RADIAN15, FtcField.RADIAN345);
  }
}
