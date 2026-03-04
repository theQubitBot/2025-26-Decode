package org.firstinspires.ftc.teamcode.qubit.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
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
  private Pose startingPose, parkingPose, goalPose, resetAudiencePose, resetGoalPose;
  private TeleOpLocalizerAsyncUpdater asyncUpdater = null;
  private Thread asyncUpdaterThread = null;
  private final Object savePositionLock = new Object();
  private final Object followerLock = new Object();
  private final double simTimeMs = 500;
  public boolean telemetryEnabled = true;
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
    updateFollower();
    Pose robotPose;
    synchronized (followerLock) {
      robotPose = follower.getPose();
    }

    return robotPose.distanceFrom(goalPose);
  }

  /**
   * Returns goal heading relative to robot pose.
   *
   * @return Goal heading in radians.
   */
  public double getGoalHeading() {
    updateFollower();
    Pose robotPose;
    synchronized (followerLock) {
      robotPose = follower.getPose();
    }

    return Math.atan2(goalPose.getY() - robotPose.getY(), goalPose.getX() - robotPose.getX());
  }

  public double getGoalHeadingInMotion() {
    updateFollower();
    double uX, uY, aX, aY;
    synchronized (followerLock) {
      uX = follower.poseTracker.getVelocity().getXComponent();
      uY = follower.poseTracker.getVelocity().getYComponent();
      aX = follower.poseTracker.getAcceleration().getXComponent();
      aY = follower.poseTracker.getAcceleration().getYComponent();
    }

    double futureX = uX * simTimeMs + 0.5 * aX * simTimeMs * simTimeMs;
    double futureY = uY * simTimeMs + 0.5 * aY * simTimeMs * simTimeMs;
    return Math.atan2(goalPose.getY() - futureY, goalPose.getX() - futureX);
  }

  public double getRobotHeadingInMotion() {
    updateFollower();
    double currentHeading, omega, headingDelta;
    synchronized (followerLock) {
      currentHeading = follower.getPose().getHeading();
      omega = follower.poseTracker.getAngularVelocity();
    }

    headingDelta = omega * simTimeMs;
    return currentHeading + headingDelta;
  }

  /**
   * Returns robot pose.
   *
   * @return Robot pose.
   */
  public Pose getRobotPose() {
    updateFollower();
    Pose robotPose;
    synchronized (followerLock) {
      robotPose = follower.getPose();
    }

    return robotPose;
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
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;

    // Initializes poses for robot starting position
    if (parent != null && parent.config != null) {
      startingPose = new Pose(parent.config.x, parent.config.y, parent.config.heading);

      // Regardless of robot starting position, parking and goal are fixed.
      if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
        parkingPose = FtcField.blueParkingPose;
        goalPose = FtcField.blueGoalPose;
        resetAudiencePose = FtcField.blueAudienceResetPose;
        resetGoalPose = FtcField.blueGoalResetPose;
      } else {
        parkingPose = FtcField.redParkingPose;
        goalPose = FtcField.redGoalPose;
        resetAudiencePose = FtcField.redAudienceResetPose;
        resetGoalPose = FtcField.redGoalResetPose;
      }
    } else {
      startingPose = new Pose(0, 0, 0);
      parkingPose = new Pose(0, 0, 0);
      goalPose = new Pose(0, 0, 0);
      resetAudiencePose = new Pose(0, 0, 0);
      resetGoalPose = new Pose(0, 0, 0);
    }

    reset(startingPose);
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
    } else if (gamePad1.shareWasPressed() || gamePad2.shareWasPressed()) {
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
    } else if (gamePad1.optionsWasPressed()) {
      reset(resetAudiencePose);
      gamePad1.rumble(500);
    } else if (gamePad2.optionsWasPressed()) {
      reset(resetGoalPose);
      gamePad2.rumble(500);
    }
    //  else Nothing to do. Pose will be  updated when needed.

    FtcLogger.exit();
  }

  public void reset(Pose resetPose) {
    synchronized (followerLock) {
      if (follower != null) {
        // clear previous follower, if any
        follower.breakFollowing();
      }

      startingPose = resetPose;
      follower = Constants.createFollower(hardwareMap);
      follower.setMaxPower(1.0); // override any autoOp set max power.
      follower.setStartingPose(startingPose);
      follower.update();
      if (parent != null && parent.driveTrain != null) {
        parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      }
    }
  }

  public boolean robotInLaunchZone() {
    Pose robotPose = getRobotPose();
    return FtcField.goalLaunchTriangle.contains(robotPose) ||
        FtcField.audienceLaunchTriangle.contains(robotPose);
  }

  public boolean robotPointingAtGoal() {
    double delta = Math.abs(getRobotPose().getHeading() - getGoalHeading());
    delta = MathFunctions.normalizeAngle(delta);
    return FtcUtils.outsideRange(delta, FtcField.RADIAN15, FtcField.RADIAN345);
  }

  public void savePosition() {
    if (follower != null) {
      updateFollower();
      Pose robotPose = getRobotPose();
      if (parent != null && parent.config != null) {
        synchronized (savePositionLock) {
          parent.config.x = robotPose.getX();
          parent.config.y = robotPose.getY();
          parent.config.heading = robotPose.getHeading();
          parent.config.saveToFile();
        }
      }
    }
  }

  /*
   * Displays telemetry. Helps with debugging.
   */
  @Override
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled && telemetry != null) {
      Pose robotPose = getRobotPose();
      Vector robotVelocity, robotAcceleration;
      double robotAngularVelocity;
      synchronized (followerLock) {
        robotVelocity = follower.poseTracker.getVelocity();
        robotAcceleration = follower.poseTracker.getAcceleration();
        robotAngularVelocity = follower.poseTracker.getAngularVelocity();
      }

      telemetry.addData("robotPose", "%.1f %.1f %.1f",
          robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
//      telemetry.addData("robotVelocity", "%.1f %.1f",
//          robotVelocity.getMagnitude(), Math.toDegrees(robotVelocity.getTheta()));
//      telemetry.addData("robotAcceleration", "%.1f %.1f",
//          robotAcceleration.getMagnitude(), Math.toDegrees(robotAcceleration.getTheta()));
//      telemetry.addData("robotAngularVelocity", "%.1f",
//          robotAngularVelocity);
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

  private void startAsyncOperations() {
    asyncUpdater = new TeleOpLocalizerAsyncUpdater(this);
    asyncUpdaterThread = new Thread(asyncUpdater, TeleOpLocalizerAsyncUpdater.TAG);
    asyncUpdaterThread.setPriority(Thread.NORM_PRIORITY + 1); // Priority 6 for position updates
    asyncUpdaterThread.setDaemon(true); // Auto-terminate on shutdown for fast OpMode transitions
    asyncUpdaterThread.start();
  }

  /**
   * Stops the localizer.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    if (follower != null) {
      synchronized (followerLock) {
        follower.breakFollowing();
        parent.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      }

      savePosition();
    }

    FtcLogger.exit();
  }

  private void stopAsyncOperations() {
    // Stop and join existing thread before creating a new one
    if (asyncUpdater != null) {
      asyncUpdater.stop();

      // Wait for thread to finish before cleaning up resources
      if (asyncUpdaterThread != null && asyncUpdaterThread.isAlive()) {
        try {
          asyncUpdaterThread.join(6);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }

      asyncUpdater = null;
      asyncUpdaterThread = null;
    }
  }

  private void updateFollower() {
    synchronized (followerLock) {
      if (follower != null) {
        follower.updatePose();
        //follower.updateErrorAndVectors();
      }
    }
  }
}
