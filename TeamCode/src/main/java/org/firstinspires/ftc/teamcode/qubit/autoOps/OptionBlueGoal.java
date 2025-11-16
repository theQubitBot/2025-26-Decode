package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueGoal extends OptionBase {
  public Pose scorePose = new Pose(-44, 0, RADIAN0);
  public Pose pickup1Pose = new Pose(-24, 29, RADIAN45);
  public Pose pickup1ControlPose = new Pose(-42, 8, RADIAN45);
  public Pose pickup2Pose = new Pose(-37, 50, RADIAN45);
  public Pose pickup2ControlPose = new Pose(-57, 25, RADIAN45);
  public Pose pickup3Pose = new Pose(-52, 57, RADIAN45);
  public Pose pickup3ControlPose = new Pose(-73, 43, RADIAN45);
  public Pose leavePose = new Pose(-44, 24, RADIAN45);

  PathChain scorePreloadPath, leavePath,
      pickup1Path, pickup2Path, pickup3Path,
      score1Path, score2Path, score3Path;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = false;
    public boolean deliverPreloaded = true,
        deliver1 = true, deliver2 = true, deliver3 = true,
        leave = true;
  }

  public static OptionBlueGoal.Params PARAMS = new OptionBlueGoal.Params();

  public OptionBlueGoal(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    ccd = robot.cannon.getClosestData(44);
  }

  public OptionBlueGoal init() {
    // preloaded artifacts
    scorePreloadPath = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) robot.aprilTag.pointAtObelisk();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) robot.intake.spinHold();
        })
        .build();

    // first artifact row
    pickup1Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup1ControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1ControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addPath(new BezierLine(pickup1ControlPose, pickup1Pose))
        .setConstantHeadingInterpolation(pickup1ControlPose.getHeading())
        .build();

    score1Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup1Pose, scorePose))
        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(500, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .build();

    // second artifact row
    pickup2Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup2ControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2ControlPose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addPath(new BezierLine(pickup2ControlPose, pickup2Pose))
        .setLinearHeadingInterpolation(pickup2ControlPose.getHeading(), pickup2Pose.getHeading())
        .build();

    score2Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup2Pose, pickup2ControlPose))
        .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2ControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addPath(new BezierLine(pickup2ControlPose, scorePose))
        .setLinearHeadingInterpolation(pickup2ControlPose.getHeading(), scorePose.getHeading())
        .build();

    // third artifact rwo
    pickup3Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup3ControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3ControlPose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addPath(new BezierLine(pickup3ControlPose, pickup3Pose))
        .setConstantHeadingInterpolation(pickup3ControlPose.getHeading())
        .build();

    score3Path = follower.pathBuilder()
        .addPath(new BezierCurve(pickup3Pose, pickup3ControlPose, scorePose))
        .setLinearHeadingInterpolation(pickup3ControlPose.getHeading(), scorePose.getHeading())
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
      if (PARAMS.executeTrajectories) runFollower(pickup1Path, true, 3000);
      if (PARAMS.executeTrajectories) FtcUtils.interruptibleSleep(1000, autoOpMode);
      if (PARAMS.executeTrajectories) runFollower(score1Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver second row
    if (!saveAndTest()) return;
    if (PARAMS.deliver2) {
      if (PARAMS.executeTrajectories) runFollower(pickup2Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score2Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver third row
    if (!saveAndTest()) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Leave
    if (!saveAndTest()) return;
    if (PARAMS.leave) {
      if (PARAMS.executeTrajectories) runFollower(leavePath, false, 3000);
    }

    FtcLogger.exit();
  }
}
