package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.qubit.core.CannonControlData;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A base class to provide common variables and methods.
 */
public class OptionBase {
  protected LinearOpMode autoOpMode;
  protected BaseBot robot;
  protected Follower follower;
  protected Pose startPose;
  protected double pickupMaxPower = 0.8;
  protected double scoreMaxPower = 1.0;
  public CannonControlData ccd, ccd3;
  protected Pose scorePose, score3Pose, leavePose, gatePose,
      pickup1Pose, pickup1ControlPose,
      pickup2Pose, pickup2ControlPose,
      pickup3Pose, pickup3ControlPose,
      pickupLandingZonePose, pickupLandingZoneControlPose;

  protected PathChain scorePreloadPath, leavePath, gatePath,
      pickup1Path, score1Path,
      pickup2Path, score2Path,
      pickup3Path, score3Path,
      pickupLandingZone1Path, pickupLandingZone2Path, scoreLandingZonePath;
  protected Runnable intakeSpinIn, intakeSpinHold,
      sorterGreen, sorterPurple, sorterStraight,
      sorterPushGreen, sorterPushPurple, cannonIdle;

  /**
   * A Base AutoOp option class to hold all the common variables.
   *
   * @param autoOpMode The autoOp itself.
   * @param robot      The robot object to execute robot actions.
   * @param follower   The PP follower to execute trajectories.
   */
  public OptionBase(LinearOpMode autoOpMode, BaseBot robot, Follower follower) {
    this.autoOpMode = autoOpMode;
    this.robot = robot;
    this.follower = follower;

    intakeSpinIn = () -> robot.intake.spinIn(false);
    intakeSpinHold = () -> robot.intake.spinHold();

    sorterGreen = () -> robot.sorter.setGreen(false);
    sorterPurple = () -> robot.sorter.setPurple(false);
    sorterStraight = () -> robot.sorter.setStraight(false);
    sorterPushGreen = () -> robot.sorter.pushGreen(false);
    sorterPushPurple = () -> robot.sorter.pushPurple(false);
    cannonIdle = () -> robot.cannon.setVelocity(FtcCannon.CANNON_IDLE_VELOCITY, false);
  }

  /**
   * Executes the path chain.
   *
   * @param pathChain The pathChain to execute
   * @param holdEnd   When true, attempts to hold the path end point. This flag is
   *                  passed onto the pathChain follower.
   * @param timeout   A user provided timeout for path execution. Path execution is
   *                  terminated if robot doesn't finish within the timeout value.
   */
  public void runFollower(PathChain pathChain, double maxPower, boolean holdEnd, long timeout) {
    FtcLogger.enter();
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    follower.followPath(pathChain, maxPower, holdEnd);
    if (timeout < 0) timeout = 5000; // a reasonable timeout for the path
    Deadline d = new Deadline(timeout, TimeUnit.MILLISECONDS);
    do {
      follower.update();
    } while (autoOpMode.opModeIsActive() && !d.hasExpired() && follower.isBusy());
    if (follower.isBusy()) follower.breakFollowing();
    String message = String.format(Locale.US, "execution: %.0f ms", runtime.milliseconds());
    FtcLogger.info(FtcUtils.TAG, message);
    autoOpMode.telemetry.addData(FtcUtils.TAG, message);
    autoOpMode.telemetry.update();
    FtcLogger.exit();
  }

  /**
   * A helper method to test if the autoOp is active. When autoOp is inactive,
   * stores the lift and gyro values for use in TeleOp.
   *
   * @return True if Op is still active.
   */
  public boolean saveAndTest(boolean forceSave) {
    FtcLogger.enter();
    boolean opModeIsActive = autoOpMode.opModeIsActive();
    if (!opModeIsActive || forceSave) {
      follower.updatePose();
      Pose robotPose = follower.getPose();
      robot.config.x = robotPose.getX();
      robot.config.y = robotPose.getY();
      robot.config.heading = robotPose.getHeading();
      robot.config.saveToFile();
    }

    FtcLogger.exit();
    return opModeIsActive;
  }

  public void showRuntime(ElapsedTime runtime) {
    autoOpMode.telemetry.addData("Elapsed time", "%.0f", runtime.seconds());
    autoOpMode.telemetry.update();
  }

  public void updateMotif() {
    ObeliskTagEnum ote = robot.aprilTag.getObeliskTag();
    if (ote != ObeliskTagEnum.UNKNOWN && ote != robot.config.obeliskTagEnum) {
      robot.config.obeliskTagEnum = ote;
      robot.config.saveToFile();
    }
  }
}
