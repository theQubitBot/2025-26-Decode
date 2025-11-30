package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.qubit.core.CannonControlData;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A base class to provide common variables and methods.
 */
public class OptionBase {
  protected static final double RADIAN0;
  protected static final double RADIAN15;
  protected static final double RADIAN20;
  protected static final double RADIAN22;
  protected static final double RADIAN30;
  protected static final double RADIAN45;
  protected static final double RADIAN60;
  protected static final double RADIAN70;
  protected static final double RADIAN75;
  protected static final double RADIAN90;
  protected static final double RADIAN105;
  protected static final double RADIAN120;
  protected static final double RADIAN133;
  protected static final double RADIAN135;
  protected static final double RADIAN150;
  protected static final double RADIAN180;
  protected LinearOpMode autoOpMode;
  public CannonControlData ccd;
  protected FtcBot robot;
  protected Follower follower;
  protected final Pose startPose = new Pose(0, 0, 0);

  protected Runnable intakeSpinIn, intakeSpinOut, intakeSpinHold,
      sorterGreen, sorterPurple, sorterStraight, cannonIdle;

  static {
    RADIAN0 = Math.toRadians(0);
    RADIAN15 = Math.toRadians(15);
    RADIAN20 = Math.toRadians(20);
    RADIAN22 = Math.toRadians(22);
    RADIAN30 = Math.toRadians(30);
    RADIAN45 = Math.toRadians(45);
    RADIAN60 = Math.toRadians(60);
    RADIAN70 = Math.toRadians(70);
    RADIAN75 = Math.toRadians(75);
    RADIAN90 = Math.toRadians(90);
    RADIAN105 = Math.toRadians(105);
    RADIAN120 = Math.toRadians(120);
    RADIAN133 = Math.toRadians(133);
    RADIAN135 = Math.toRadians(135);
    RADIAN150 = Math.toRadians(150);
    RADIAN180 = Math.toRadians(179.99);
  }

  /**
   * A Base AutoOp option class to hold all the common variables.
   *
   * @param autoOpMode The autoOp itself.
   * @param robot      The robot object to execute robot actions.
   * @param follower   The PP follower to execute trajectories.
   */
  public OptionBase(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    this.autoOpMode = autoOpMode;
    this.robot = robot;
    this.follower = follower;

    intakeSpinIn = () -> robot.intake.spinIn(false);
    intakeSpinOut = () -> robot.intake.spinOut(false);
    intakeSpinHold = () -> robot.intake.spinHold();

    sorterGreen = () -> robot.sorter.setGreen(false);
    sorterPurple = () -> robot.sorter.setPurple(false);
    sorterStraight = () -> robot.sorter.setStraight(false);
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
  public void runFollower(PathChain pathChain, boolean holdEnd, long timeout) {
    FtcLogger.enter();
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    follower.followPath(pathChain, holdEnd);
    if (timeout < 0) timeout = Long.MAX_VALUE;
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
  public boolean saveAndTest() {
    FtcLogger.enter();
    boolean opModeIsActive;
    if (autoOpMode.opModeIsActive()) {
      opModeIsActive = true;
    } else {
      opModeIsActive = false;
    }

    FtcLogger.exit();
    return opModeIsActive;
  }
}
