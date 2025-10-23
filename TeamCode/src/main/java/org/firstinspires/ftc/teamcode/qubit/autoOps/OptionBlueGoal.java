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
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueGoal extends OptionBase {
  public Pose scorePose = new Pose(-45, 0, RADIAN0);
  public Pose pickup1Pose = new Pose(-32, 24, RADIAN45);
  public Pose pickup1ControlPose = new Pose(-32, 24, RADIAN45);
  public Pose pickup2Pose = new Pose(-46, 43, RADIAN45);
  public Pose pickup2ControlPose = new Pose(-46, 43, RADIAN45);
  public Pose pickup3Pose = new Pose(-61, 60, RADIAN45);
  public Pose pickup3ControlPose = new Pose(-61, 60, RADIAN45);
  public Pose leavePose = new Pose(-35, 36, -RADIAN45);

  PathChain scorePreloadPath, leavePath,
      pickup1Path, pickup2Path, pickup3Path,
      score1Path, score2Path, score3Path;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = false;
    public boolean deliverPreloaded = true,
        deliver1 = false, deliver2 = false, deliver3 = false,
        leave = false;
  }

  public static OptionBlueGoal.Params PARAMS = new OptionBlueGoal.Params();

  public OptionBlueGoal(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    cpd = robot.cannon.powerData.get(1);
  }

  public OptionBlueGoal init() {
    // preloaded artifacts
    scorePreloadPath = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setPower(cpd.power);
        })
        .build();

    // first artifact row
    pickup1Path = follower.pathBuilder()
        .addPath(new BezierCurve(scorePose, pickup1ControlPose, pickup1Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
        .addTemporalCallback(200, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score1Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup1Pose, scorePose))
        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(20, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(30, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setPower(cpd.power);
        })
        .build();

    // second artifact row
    pickup2Path = follower.pathBuilder()
        .addPath(new BezierCurve(scorePose, pickup2ControlPose, pickup2Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score2Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup2Pose, scorePose))
        .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(20, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(30, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setPower(cpd.power);
        })
        .build();

    // third artifact rwo
    pickup3Path = follower.pathBuilder()
        .addPath(new BezierCurve(scorePose, pickup3ControlPose, pickup3Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score3Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup3Pose, scorePose))
        .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(20, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(30, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setPower(cpd.power);
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
      FtcUtils.sleep(250);
      ObeliskTagEnum ote = robot.aprilTag.getObeliskTag();
      if (ote != ObeliskTagEnum.UNKNOWN) {
        robot.config.obeliskTagEnum = ote;
        robot.config.saveToFile();
      }

      if (PARAMS.executeRobotActions) robot.cannon.fire(cpd, robot.config.obeliskTagEnum);
    }

    // Deliver first row
    if (!saveAndTest()) return;
    if (PARAMS.deliver1) {
      if (PARAMS.executeTrajectories) runFollower(pickup1Path, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score1Path, true, 2500);
      if (PARAMS.executeRobotActions) robot.cannon.fire(cpd, robot.config.obeliskTagEnum);
    }

    // Deliver second row
    if (!saveAndTest()) return;
    if (PARAMS.deliver2) {
      if (PARAMS.executeTrajectories) runFollower(pickup2Path, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score2Path, true, 2500);
      if (PARAMS.executeRobotActions) robot.cannon.fire(cpd, robot.config.obeliskTagEnum);
    }

    // Deliver third row
    if (!saveAndTest()) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 2500);
      if (PARAMS.executeRobotActions) robot.cannon.fire(cpd, robot.config.obeliskTagEnum);
    }

    // Leave
    if (!saveAndTest()) return;
    if (PARAMS.leave) {
      if (PARAMS.executeTrajectories) runFollower(leavePath, false, 4000);
    }

    FtcLogger.exit();
  }
}
