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
public class OptionBlueAudience extends OptionBase {
  Pose scorePose = startPose;

  public Pose pickup3Pose = new Pose(42, 27, RADIAN70);
  public Pose pickup3ControlPose = new Pose(24, 1, RADIAN70);
  public Pose pickupLoadingZonePose = new Pose(25, 35, RADIAN75);
  public Pose pickupLoadingZoneControlPose = new Pose(15, 7, RADIAN75);
  public Pose leavePose = new Pose(21, 0, RADIAN0);

  PathChain leavePath,
      pickup3Path, pickupLoadingZonePath,
      score3Path, scoreLoadingZonePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver3 = true, deliverLoadingZone = true, leave = true;
  }

  public static OptionBlueAudience.Params PARAMS = new OptionBlueAudience.Params();

  public OptionBlueAudience(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    ccd = robot.cannon.getClosestData(113);
  }

  public OptionBlueAudience init() {
    // third artifact rwo
    pickup3Path = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup3ControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3ControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addPath(new BezierLine(pickup3ControlPose, pickup3Pose))
        .setConstantHeadingInterpolation(pickup3ControlPose.getHeading())
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

    pickupLoadingZonePath = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickupLoadingZoneControlPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLoadingZoneControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addPath(new BezierLine(pickupLoadingZoneControlPose, pickupLoadingZonePose))
        .build();

    scoreLoadingZonePath = follower.pathBuilder()
        .addPath(new BezierLine(pickupLoadingZonePose, pickupLoadingZoneControlPose))
        .setLinearHeadingInterpolation(pickupLoadingZonePose.getHeading(), pickupLoadingZoneControlPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, false);
        })
        .addTemporalCallback(5, () -> {
          if (PARAMS.executeRobotActions) intakeSpinHold.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) sorterStraight.run();
        })
        .addPath(new BezierLine(pickupLoadingZoneControlPose, scorePose))
        .setConstantHeadingInterpolation(scorePose.getHeading())
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
      if (PARAMS.executeRobotActions) robot.intake.spinHold();
      if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, true);
      if (PARAMS.executeRobotActions) robot.cannon.setVelocity(ccd.velocity, true);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver third trow
    if (!saveAndTest()) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, false, 3000);
      if (PARAMS.executeTrajectories) FtcUtils.interruptibleSleep(2000, autoOpMode);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 3000);
      if (PARAMS.executeRobotActions)
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Deliver loading zone
    if (!saveAndTest()) return;
    if (PARAMS.deliverLoadingZone) {
      if (PARAMS.executeTrajectories) runFollower(pickupLoadingZonePath, false, 3000);
      if (PARAMS.executeTrajectories) runFollower(scoreLoadingZonePath, true, 3000);
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
