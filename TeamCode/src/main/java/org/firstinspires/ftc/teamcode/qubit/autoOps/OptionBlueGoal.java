package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.CannonControlData;
import org.firstinspires.ftc.teamcode.qubit.core.Field.FtcField;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueGoal extends OptionBase {
  public Pose scorePose = new Pose(20.5, 18.5, FtcField.RADIAN45);
  public Pose score3Pose = new Pose(44, 15, FtcField.RADIAN67);
  public Pose pickup1Pose = new Pose(14.5, 52.5, FtcField.RADIAN90);
  public Pose pickup1ControlPose = new Pose(14.5, 26, FtcField.RADIAN90);
  public Pose pickup2Pose = new Pose(-10.5, 60.5, FtcField.RADIAN90);
  public Pose pickup2ControlPose = new Pose(-10.5, 25.5, FtcField.RADIAN90);
  public Pose pickup3Pose = new Pose(-33.5, 60.5, FtcField.RADIAN90);
  public Pose pickup3ControlPose = new Pose(-33.5, 24.5, FtcField.RADIAN90);
  public Pose leavePose = new Pose(30, 50, FtcField.RADIAN45);

  PathChain scorePreloadPath,
      pickup1Path, score1Path,
      pickup2Path, score2Path,
      pickup3Path, score3Path,
      leavePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver1 = true, deliver2 = true, deliver3 = true,
        leave = true;
  }

  public static OptionBlueGoal.Params PARAMS = new OptionBlueGoal.Params();

  public OptionBlueGoal(LinearOpMode autoOpMode, BaseBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    startPose = FtcField.blueGoalStartPose;
    follower.setStartingPose(startPose);
    ccd = CannonControlData.getClosestData(CannonControlData.GOAL_SWEET_SPOT_DISTANCE);
    ccd3 = CannonControlData.getClosestData(CannonControlData.GOAL_SWEET_SPOT_DISTANCE3);
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
    pickup1Path = follower.pathBuilder()
        .addPath(BezierCurve.through(scorePose, pickup1ControlPose, pickup1Pose))
        .setHeadingInterpolation(HeadingInterpolator.piecewise(
            new HeadingInterpolator.PiecewiseNode(
                0,
                .2,
                HeadingInterpolator.linear(scorePose.getHeading(), pickup1ControlPose.getHeading())
            ),
            new HeadingInterpolator.PiecewiseNode(
                .21,
                1,
                HeadingInterpolator.constant(pickup1ControlPose.getHeading())
            ))
        )
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) sorterPurple.run();
        })
        .addTemporalCallback(500, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
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
        .addTemporalCallback(1000, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .build();

    // second artifact row
    pickup2Path = follower.pathBuilder()
        .addPath(BezierCurve.through(scorePose, pickup2ControlPose, pickup2Pose))
        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                    0,
                    .29,
                    HeadingInterpolator.linear(scorePose.getHeading(), pickup2ControlPose.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                    .3,
                    1,
                    HeadingInterpolator.constant(pickup2ControlPose.getHeading())
                )
            )
        )
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) sorterPurple.run();
        })
        .addTemporalCallback(1000, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score2Path = follower.pathBuilder()
        .addPath(BezierCurve.through(pickup2Pose, pickup2ControlPose, scorePose))
        .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(1000, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .build();

    // third artifact row
    pickup3Path = follower.pathBuilder()
        .addPath(BezierCurve.through(scorePose, pickup3ControlPose, pickup3Pose))
        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                    0, .45, HeadingInterpolator.linear(scorePose.getHeading(), pickup3ControlPose.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                    .46, 1, HeadingInterpolator.constant(pickup3ControlPose.getHeading())
                )
            )
        )
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) sorterGreen.run();
        })
        .addTemporalCallback(1000, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score3Path = follower.pathBuilder()
        .addPath(new BezierCurve(pickup3Pose, pickup3ControlPose, score3Pose))
        .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd3.velocity, false);
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(1000, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
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
  public void execute(ElapsedTime runtime) {
    FtcLogger.enter();

    // Deliver preloaded artifacts
    if (!saveAndTest(false)) return;

    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, scoreMaxPower, true, 3000);
      updateMotif();
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver first row
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver1) {
      if (PARAMS.executeTrajectories) runFollower(pickup1Path, pickupMaxPower, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score1Path, scoreMaxPower, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver second row
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver2 && robot.config.deliverSecondRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup2Path, pickupMaxPower, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score2Path, scoreMaxPower, false, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver third row
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, pickupMaxPower, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score3Path, scoreMaxPower, false, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd3, robot.config.obeliskTagEnum, autoOpMode);
    } else {
      // Leave
      if (!saveAndTest(false)) return;
      if (PARAMS.leave) {
        if (PARAMS.executeTrajectories) runFollower(leavePath, pickupMaxPower, false, 3000);
      }
    }

    FtcLogger.exit();
  }
}
