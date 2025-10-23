package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueAudience extends OptionBase {
  Pose scorePose = startPose;

  public Pose pickup3Pose = new Pose(15, 20, RADIAN70);
  public Pose pickup3ControlPose = new Pose(15, 20, RADIAN70);
  public Pose pickupLoadingZonePose = new Pose(15, 20, RADIAN133);
  public Pose pickupLoadingZoneControlPose = new Pose(15, 20, RADIAN133);

  public Pose leavePose = new Pose(-24, 12, -RADIAN20);

  PathChain leavePath,
      pickup3Path, pickupLoadingZonePath,
      score3Path, scoreLoadingZonePath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = false;
    public boolean deliverPreloaded = true,
        deliver3 = false, deliverLoadingZone = false, leave = false;
  }

  public static OptionBlueAudience.Params PARAMS = new OptionBlueAudience.Params();

  public OptionBlueAudience(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
    cpd = robot.cannon.powerData.get(robot.cannon.powerData.size() - 1);
  }

  public OptionBlueAudience init() {
    // third artifact rwo
    pickup3Path = follower.pathBuilder()
        .addPath(new BezierCurve(scorePose,pickup3ControlPose, pickup3Pose))
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
        .addPath(new BezierCurve(scorePose,pickupLoadingZoneControlPose, pickupLoadingZonePose))
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

    if (!saveAndTest()) return;

    // Deliver preloaded
    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeRobotActions) robot.cannon.fire(cpd, robot.config.obeliskTagEnum);
    }

    // Deliver third trow
    if (!saveAndTest()) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) runFollower(pickup3Path, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score3Path, true, 2500);
      if (PARAMS.executeRobotActions) robot.cannon.fire(cpd, robot.config.obeliskTagEnum);
    }

    // Deliver loading zone
    if (!saveAndTest()) return;
    if (PARAMS.deliverLoadingZone) {
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
