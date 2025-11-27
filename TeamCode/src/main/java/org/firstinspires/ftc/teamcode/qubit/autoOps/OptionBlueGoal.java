package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueGoal extends OptionBase {
  public Pose scorePose = new Pose(-44, 0, RADIAN0);
  public Pose pickup1Pose = new Pose(-24, 29, RADIAN45);
  public Pose pickup1ControlPose = new Pose(-42, 10, RADIAN45);
  public Pose pickup2Pose = new Pose(-35, 50, RADIAN45);
  public Pose pickup2ControlPose = new Pose(-59, 27, RADIAN45);
  public Pose leavePose = new Pose(-44, 12, RADIAN0);

  PathChain scorePreloadPath,
      pickup11Path, pickup12Path, score1Path,
      pickup21Path, pickup22Path, score21Path, score22Path,
      leavePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver1 = true, deliver2 = true,
        leave = true;
  }

  public static OptionBlueGoal.Params PARAMS = new OptionBlueGoal.Params();

  public OptionBlueGoal(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    ccd = robot.cannon.getClosestData(FtcCannon.GOAL_SWEET_SPOT_DISTANCE);
  }

  public OptionBlueGoal init() {
    // preloaded artifacts
    scorePreloadPath = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.intake.spinHold();
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .build();

    // first artifact row
    pickup11Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup1ControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1ControlPose.getHeading())
        .build();

    pickup12Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup1ControlPose, pickup1Pose))
        .setConstantHeadingInterpolation(pickup1ControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addParametricCallback(0.9, () -> {
          if (PARAMS.executeRobotActions) sorterGreen.run();
        })
        .build();

    score1Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup1Pose, scorePose))
        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .build();

    // second artifact row
    pickup21Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup2ControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2ControlPose.getHeading())
        .build();

    pickup22Path = follower.pathBuilder()
        .addPath(new BezierCurve(pickup2ControlPose, pickup2Pose))
        .setConstantHeadingInterpolation(pickup2ControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score21Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup2Pose, pickup2ControlPose))
        .setConstantHeadingInterpolation(pickup2Pose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .build();

    score22Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup2ControlPose, scorePose))
        .setLinearHeadingInterpolation(pickup2ControlPose.getHeading(), scorePose.getHeading())
        .build();

    // leave
    leavePath = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, leavePose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
        .build();

    return this;
  }

  /**
   * Executes the autonomous workflow.
   */
  public void execute() {
    FtcLogger.enter();

    // Deliver preloaded artifacts
    if (!saveAndTest()) return;

    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 3000);
      ObeliskTagEnum ote = robot.aprilTag.getObeliskTag();
      if (ote != ObeliskTagEnum.UNKNOWN) {
        robot.config.obeliskTagEnum = ote;
        robot.config.saveToFile();
      }

      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver first row
    if (!saveAndTest()) return;
    if (PARAMS.deliver1) {
      if (PARAMS.executeTrajectories) runFollower(pickup11Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(pickup12Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score1Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver second row
    if (!saveAndTest()) return;
    if (PARAMS.deliver2) {
      if (PARAMS.executeTrajectories) runFollower(pickup21Path, true, 3000);
//      if (PARAMS.executeTrajectories) runFollower(pickup22Path, true, 3000);
//      if (PARAMS.executeTrajectories) runFollower(score21Path, false, 3000);
//      if (PARAMS.executeTrajectories) runFollower(score22Path, true, 3000);
//      if (PARAMS.executeRobotActions)
//        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Leave
    if (!saveAndTest()) return;
    if (PARAMS.leave) {
//      if (PARAMS.executeTrajectories) runFollower(leavePath, false, 3000);
    }

    FtcLogger.exit();
  }
}
