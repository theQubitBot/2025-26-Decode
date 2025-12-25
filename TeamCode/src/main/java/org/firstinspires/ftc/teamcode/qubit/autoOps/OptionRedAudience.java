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
public class OptionRedAudience extends OptionBase {
  public Pose scorePose = new Pose(3, 0, -RADIAN22);
  public Pose pickup3Pose = new Pose(28.5, -44.7, -RADIAN90);
  public Pose pickup3ControlPose = new Pose(29.5, -10.8, -RADIAN90);
  public Pose pickupLandingZonePose = new Pose(5, -47.3, -RADIAN180);
  public Pose pickupLandingZoneControlPose = new Pose(23, -38, -RADIAN135);
  public Pose leavePose = new Pose(4, -20, RADIAN0);

  PathChain scorePreloadPath,
      pickup3Path, score3Path,
      pickupLandingZone1Path, pickupLandingZone2Path, scoreLandingZonePath,
      leavePath;

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
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) robot.intake.spinHold();
        })

        .build();

    // third artifact row
    pickup3Path = follower.pathBuilder()
        .addPath(BezierCurve.through(scorePose, pickup3ControlPose, pickup3Pose))
        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                    0, .2, HeadingInterpolator.linear(scorePose.getHeading(), pickup3ControlPose.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                    .21, 1, HeadingInterpolator.constant(pickup3ControlPose.getHeading())
                )
            )
        )
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) sorterGreen.run();
        })
        .addTemporalCallback(2, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score3Path = follower.pathBuilder()
        .addPath(new BezierLine(pickup3Pose, scorePose))
        .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
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

    pickupLandingZone1Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickupLandingZoneControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLandingZoneControlPose.getHeading())
        .build();

    pickupLandingZone2Path = follower.pathBuilder()
        .addPath(new BezierLine(pickupLandingZoneControlPose, pickupLandingZonePose))
        .setLinearHeadingInterpolation(pickupLandingZoneControlPose.getHeading(), pickupLandingZonePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) sorterPurple.run();
        })
        .addTemporalCallback(2, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    scoreLandingZonePath = follower.pathBuilder()
        .addPath(new BezierLine(pickupLandingZonePose, scorePose))
        .setLinearHeadingInterpolation(pickupLandingZonePose.getHeading(), scorePose.getHeading())
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

    if (!saveAndTest(false)) return;

    // Deliver preloaded
    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 3000);
      if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, true);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver third trow
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);

      if (PARAMS.executeTrajectories) runFollower(pickupLandingZone1Path, true, 3000);
      if (PARAMS.executeTrajectories) runFollower(pickupLandingZone2Path, false, 3000);
      if (PARAMS.executeTrajectories) runFollower(scoreLandingZonePath, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Leave
    if (!saveAndTest(false)) return;
    if (PARAMS.leave) {
      if (PARAMS.executeTrajectories) runFollower(leavePath, true, 3000);
    }

    FtcLogger.exit();
  }
}
