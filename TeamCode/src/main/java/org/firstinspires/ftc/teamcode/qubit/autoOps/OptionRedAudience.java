package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class to implement autonomous objective
 */
public class OptionRedAudience extends OptionBase {
  public Pose scorePose = new Pose(2, 2, -RADIAN22);
  public Pose pickup3Pose = new Pose(30, -44, -RADIAN90);
  public Pose pickup3ControlPose = new Pose(30, -12, -RADIAN90);
  public Pose leavePose = new Pose(4, -20, RADIAN0);

  PathChain scorePreloadPath,
      pickup31Path, pickup32Path, score3Path, leavePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver3 = true, leave = true;
  }

  public static OptionRedAudience.Params PARAMS = new OptionRedAudience.Params();

  public OptionRedAudience(LinearOpMode autoOpMode, BaseBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    ccd = robot.cannon.getClosestData(FtcCannon.AUDIENCE_DISTANCE);
  }

  public OptionRedAudience init() {
    // preloaded artifacts
    scorePreloadPath = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.intake.spinHold();
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .build();

    // third artifact rwo
    pickup31Path = follower.pathBuilder()
        .addPath(new BezierLine(startPose, pickup3ControlPose))
        .setLinearHeadingInterpolation(startPose.getHeading(), pickup3ControlPose.getHeading())
        .build();

    pickup32Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup3ControlPose, pickup3Pose))
        .setConstantHeadingInterpolation(pickup3ControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) sorterGreen.run();
        })
        .build();

    score3Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup3Pose, scorePose))
        .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
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
        .setConstantHeadingInterpolation(scorePose.getHeading())
        .build();

    return this;
  }

  /**
   * Executes the autonomous workflow.
   */
  public void execute() {
    FtcLogger.enter();

    if (!saveAndTest()) return;

    // Deliver preloaded
    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 3000);
      if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, true);
      if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, true);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver third trow
    if (!saveAndTest()) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup31Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(pickup32Path, false, 3000);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Leave
    if (!saveAndTest()) return;
    if (PARAMS.leave) {
      if (PARAMS.executeTrajectories) runFollower(leavePath, true, 3000);
    }

    FtcLogger.exit();
  }
}
