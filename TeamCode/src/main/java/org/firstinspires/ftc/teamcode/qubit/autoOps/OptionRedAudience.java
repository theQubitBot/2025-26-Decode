package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.CannonControlData;
import org.firstinspires.ftc.teamcode.qubit.core.Field.FtcField;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class to implement autonomous objective
 */
public class OptionRedAudience extends OptionBase {
  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver3 = true, leave = true;
  }

  public static OptionRedAudience.Params PARAMS = new OptionRedAudience.Params();

  public OptionRedAudience(LinearOpMode autoOpMode, BaseBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    startPose = FtcField.redAudienceStartPose;
    follower.setStartingPose(startPose);
    ccd = CannonControlData.getClosestData(CannonControlData.AUDIENCE_DISTANCE);

    scorePose = new Pose(-56, -16, -FtcField.RADIAN22);
    pickup3Pose = new Pose(-33, -60.5, -FtcField.RADIAN90);
    pickup3ControlPose = new Pose(-33, -24.5, -FtcField.RADIAN90);
    pickupLandingZonePose = new Pose(-56, -62, -FtcField.RADIAN90);
    leavePose = new Pose(-62, -39, FtcField.RADIAN0);
  }

  public OptionRedAudience init() {
    if (PARAMS.executeRobotActions) robot.cannon.setHoodPosition(ccd);

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
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addTemporalCallback(2, () -> {
          if (PARAMS.executeRobotActions) sorterGreen.run();
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
        .addPath(new BezierLine(scorePose, pickupLandingZonePose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLandingZonePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addTemporalCallback(2, () -> {
          if (PARAMS.executeRobotActions) sorterPurple.run();
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
  public void execute(ElapsedTime runtime) {
    FtcLogger.enter();
    if (!saveAndTest(false)) return;
    if (PARAMS.executeRobotActions) robot.cannon.setHoodPosition(ccd);

    // Deliver preloaded
    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, scoreMaxPower, true, 3000);
      if (PARAMS.executeRobotActions) {
        robot.cannon.setVelocity(ccd.velocity, true);
        robot.cannon.setVelocity(ccd.velocity, true);
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
      }
    }

    // Deliver third trow
    if (!saveAndTest(false)) return;
    if (PARAMS.deliver3 && robot.config.deliverThirdRow) {
      if (PARAMS.executeTrajectories) {
        runFollower(pickup3Path, pickupMaxPower, true, 3000);
        runFollower(score3Path, scoreMaxPower, true, 3000);
      }

      if (PARAMS.executeRobotActions) {
        robot.cannon.setVelocity(ccd.velocity, true);
        robot.cannon.setVelocity(ccd.velocity, true);
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
      }
    } else {
      if (PARAMS.executeTrajectories) {
        runFollower(pickupLandingZone1Path, pickupMaxPower, true, 3000);
        turnTo(-FtcField.RADIAN170, 2000);
        runFollower(scoreLandingZonePath, scoreMaxPower, true, 3000);
      }

      if (PARAMS.executeRobotActions) {
        robot.cannon.setVelocity(ccd.velocity, true);
        robot.cannon.setVelocity(ccd.velocity, true);
        robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
      }
    }

    // Landing zone
    if (PARAMS.executeTrajectories) {
      runFollower(pickupLandingZone1Path, pickupMaxPower, true, 3000);
      turnTo(-FtcField.RADIAN170, 2000);
      runFollower(scoreLandingZonePath, scoreMaxPower, true, 3000);
    }

    if (PARAMS.executeRobotActions) {
      robot.cannon.setVelocity(ccd.velocity, true);
      robot.cannon.setVelocity(ccd.velocity, true);
      robot.cannon.fire(ccd, robot.config.obeliskTagEnum, autoOpMode);
    }

    // Leave
    if (!saveAndTest(false)) return;
    if (PARAMS.leave) {
      if (PARAMS.executeTrajectories) runFollower(leavePath, scoreMaxPower, true, 3000);
    }

    FtcLogger.exit();
  }
}
