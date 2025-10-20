package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueSmallTriangle extends OptionBase {
  Pose scorePose = startPose;

  public Pose pickup3Pose = new Pose(15, 20, RADIAN45);
  public Pose pickupLoadingZonePose = new Pose(15, 20, RADIAN45);

  public Pose leavePose = new Pose(-24, 12, -RADIAN0);

  PathChain leavePath,
      pickup3Path, pickupLoadingZonePath,
      score3Path, scoreLoadingZonePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = false;
    public boolean deliverPreloaded = true,
        deliver3 = false, deliverLoadingZone = false,        leave = false;
  }

  public static OptionBlueSmallTriangle.Params PARAMS = new OptionBlueSmallTriangle.Params();

  public OptionBlueSmallTriangle(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
  }

  public OptionBlueSmallTriangle init() {
    // third artifact rwo
    pickup3Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup3Pose))
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
          if (PARAMS.executeRobotActions) cannonWarmUp.run();
        })
        .build();

    pickupLoadingZonePath = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickupLoadingZonePose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLoadingZonePose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    scoreLoadingZonePath = follower.pathBuilder()
        .addPath(new BezierLine(pickupLoadingZonePose, scorePose))
        .setLinearHeadingInterpolation(pickupLoadingZonePose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(20, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addTemporalCallback(30, () -> {
          if (PARAMS.executeRobotActions) cannonWarmUp.run();
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

    // Deliver preloaded
    if (!saveAndTest()) return;

    // Deliver third trow
    if (!saveAndTest()) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 2500);
      if (PARAMS.executeRobotActions) robot.cannon.fire(robot.config.obeliskTagEnum);
    }

    // Deliver loading zone
    if (!saveAndTest()) return;
    if (PARAMS.deliverLoadingZone) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 2500);
      if (PARAMS.executeRobotActions) robot.cannon.fire(robot.config.obeliskTagEnum);
    }

    // Leave
    if (!saveAndTest()) return;
    if (PARAMS.leave) {
      if (PARAMS.executeTrajectories) runFollower(leavePath, false, 4000);
    }

    FtcLogger.exit();
  }
}
