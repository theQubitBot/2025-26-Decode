package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class to implement autonomous objective
 */
public class OptionRedGoal extends OptionBase {
  public Pose scorePose = new Pose(-44, 0, RADIAN0);
  public Pose score3Pose = new Pose(-32, 18, -RADIAN22);
  public Pose pickup1Pose = new Pose(-24.2, -28.4, -RADIAN45);
  public Pose pickup1ControlPose = new Pose(-42, -10, -RADIAN45);
  public Pose pickup2Pose = new Pose(-35.6, -51.2, -RADIAN45);
  public Pose pickup2ControlPose = new Pose(-59.5, -27.9, -RADIAN45);
  public Pose pickup3Pose = new Pose(-52.7, -66.9, -RADIAN45);
  public Pose pickup3ControlPose = new Pose(-75.7, -44.7, -RADIAN45);
  public Pose leavePose = new Pose(-24, -13, RADIAN0);

  PathChain scorePreloadPath,
      pickup11Path, score1Path,
      pickup21Path, score21Path,
      pickup31Path, score31Path,
      leavePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver1 = true, deliver2 = true, deliver3 = true,
        leave = true;
  }

  public static OptionRedGoal.Params PARAMS = new OptionRedGoal.Params();

  public OptionRedGoal(LinearOpMode autoOpMode, BaseBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    ccd = robot.cannon.getClosestData(FtcCannon.GOAL_SWEET_SPOT_DISTANCE);
    ccd3 = robot.cannon.getClosestData(FtcCannon.GOAL_SWEET_SPOT_DISTANCE3);
  }

  public OptionRedGoal init() {
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
    pickup21Path = follower.pathBuilder()
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

    score21Path = follower.pathBuilder()
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
    pickup31Path = follower.pathBuilder()
        .addPath(BezierCurve.through(scorePose, pickup3ControlPose, pickup3Pose))
        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                    0, .39, HeadingInterpolator.linear(scorePose.getHeading(), pickup3ControlPose.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                    .4, 1, HeadingInterpolator.constant(pickup3ControlPose.getHeading())
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

    score31Path = follower.pathBuilder()
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
  public void execute() {
    FtcLogger.enter();

    // Deliver preloaded artifacts
    if (!saveAndTest(false)) return;

    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 3000);
      updateMotif();
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver first row
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver1) {
      if (PARAMS.executeTrajectories) runFollower(pickup11Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score1Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver second row
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver2 && robot.config.deliverSecondRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup21Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score21Path, false, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver third row
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup31Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score31Path, false, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd3, robot.config.obeliskTagEnum, autoOpMode);
    } else {
      // Leave
      if (!saveAndTest(false)) return;
      if (PARAMS.leave) {
        if (PARAMS.executeTrajectories) runFollower(leavePath, false, 3000);
      }
    }

    FtcLogger.exit();
  }
}
