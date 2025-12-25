package org.firstinspires.ftc.teamcode.qubit.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

/**
 * A class to manage robot operations in Tele Op using Pedro Pathing
 */
public class TeleOpLocalizer extends FtcSubSystemBase {
  private static final String TAG = "TeleOpLocalizer";
  private Follower follower = null; // set in init
  private Pose startingPose = new Pose(0, 0, 0); // default, overridden in init
  private final Pose parkPoseForBlueAudience = new Pose(25.5, -50.5, Math.toRadians(90));
  private final Pose parkPoseForBlueGoal = new Pose(-121, 1.6, Math.toRadians(45));
  private final Pose parkPoseForRedAudience = new Pose(27.2, 50.5, Math.toRadians(-90));
  private final Pose parkPoseForRedGoal = new Pose(-121, -1.6, Math.toRadians(-45));
  private Pose parkingPose = parkPoseForBlueGoal; // default, overridden in init
  private final Pose tagPoseForBlueAudience = new Pose(128, 47, Math.toRadians(45));
  private final Pose tagPoseForBlueGoal = new Pose(8, 0, 0);
  private final Pose tagPoseForRedAudience = new Pose(128, -47, Math.toRadians(-45));
  private final Pose tagPoseForRedGoal = new Pose(8, 0, 0);
  private Pose tagPose = tagPoseForBlueGoal; // default, overridden in init

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
    Pose roboPose = follower.getPose();
    return roboPose.distanceFrom(tagPose);
  }

  /**
   * Returns goal heading relative to robot pose.
   *
   * @return Goal heading in radians.
   */
  public double getGoalHeading() {
    double heading = 0;
    follower.updatePose();
    Pose robotPose = follower.getPose();
    heading = MathFunctions.normalizeAngle(Math.atan2(
        tagPose.getY() - robotPose.getY(), tagPose.getX() - robotPose.getX()));
    return heading;
  }

  /**
   * Returns robot heading.
   *
   * @return Robot heading in radians.
   */
  public double getRobotHeading() {
    follower.updatePose();
    return follower.getPose().getHeading();
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
      if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
        if (parent.config.robotPosition == RobotPositionEnum.GOAL) {
          parkingPose = parkPoseForBlueGoal;
          tagPose = tagPoseForBlueGoal;
        } else {
          parkingPose = parkPoseForBlueAudience;
          tagPose = tagPoseForBlueAudience;
        }
      } else {
        if (parent.config.robotPosition == RobotPositionEnum.GOAL) {
          parkingPose = parkPoseForRedGoal;
          tagPose = tagPoseForRedGoal;
        } else {
          parkingPose = parkPoseForRedAudience;
          tagPose = tagPoseForRedAudience;
        }
      }
    }

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startingPose);
    follower.update();
    FtcLogger.exit();
  }

  /**
   * Operates the cannon using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  @Override
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();
    if (gamePad1.dpadUpWasPressed() || gamePad2.dpadUpWasPressed()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      follower.turnTo(getGoalHeading());
    } else if (gamePad1.dpad_up || gamePad2.dpad_up) {
      follower.update();
    } else if (gamePad1.dpadUpWasReleased() || gamePad2.dpadUpWasReleased()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
    } else if (gamePad1.dpadDownWasPressed() || gamePad2.dpadDownWasPressed()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
      follower.updatePose();
      follower.holdPoint(follower.getPose());
    } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
      follower.update();
    } else if (gamePad1.dpadDownWasReleased() || gamePad2.dpadDownWasReleased()) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
    }
    //  else Nothing to do. Pose will be  updated when needed.

    if (FtcUtils.lastNSeconds(runtime, FtcUtils.ENDGAME_PARK_WARNING_SECONDS)) {
      if (gamePad1.shareWasPressed() || gamePad2.shareWasPressed()) {
        if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
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
      }
    }

    FtcLogger.exit();
  }

  /*
   * Displays cannon telemetry. Helps with debugging.
   */
  @Override
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled && telemetry != null) {
      follower.updatePose();
      Pose pose = follower.getPose();
      telemetry.addData("Pose", "%.1f %.1f %.1f", pose.getX(), pose.getY(), pose.getHeading());
      telemetry.addData("Goal distance", "%.1f", getGoalDistance());
      telemetry.addData("Goal heading", "%.1f", Math.toDegrees(getGoalHeading()));
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
   * Stops the cannon.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    if (follower != null) {
      if (follower.isBusy() || follower.isTurning()) follower.breakFollowing();
    }

    FtcLogger.exit();
  }

  public boolean isValidShootingDistance() {
    double distance = getGoalDistance();
    return (distance >= CannonControlData.L30 && distance <= CannonControlData.L75) ||
        (distance >= CannonControlData.L100 && distance <= CannonControlData.L120);
  }

  public boolean isValidShootingHeading() {
    return Math.abs(Math.toDegrees(getRobotHeading() - getGoalHeading())) <= 15;
  }
}
