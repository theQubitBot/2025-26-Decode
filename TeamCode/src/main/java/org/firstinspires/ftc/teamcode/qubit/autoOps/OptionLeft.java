package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

/**
 * A class to implement autonomous objective
 */
@Config
public class OptionLeft extends OptionBase {
  Pose scorePose = new Pose(-4, 13.5, RADIAN0);

  // First yellow Sample
  public Pose pickup1Pose = new Pose(15, 20, RADIAN45);

  // Second yellow Sample
  public Pose pickup2Pose = new Pose(7, 28, RADIAN45);

  // Third yellow Sample

  public Pose pickup3Pose = new Pose(9, 23.5, RADIAN90);

  // Park
  public Pose parkControlPose = new Pose(26, 47.3, -RADIAN15);
  public Pose parkPose = new Pose(77, 9, -RADIAN45);

  PathChain scorePreloadPath, parkPath,
      pickup1, pickup2, pickup3, score1, score2, score3;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = true;
    public boolean deliverPreloaded = true,
        deliver1 = true, deliver2 = true, deliver3 = true,
        park = true;
  }

  public static Params PARAMS = new Params();

  public OptionLeft(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
  }

  public OptionLeft init() {
    // preloaded sample
    scorePreloadPath = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDown.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinStop.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
          if (PARAMS.executeRobotActions) lift2HighBasket.run();
        })
        .build();

    // first yellow
    pickup1 = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup1Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) lift2Low.run();
        })
        .addTemporalCallback(150, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDown.run();
        })
        .addTemporalCallback(200, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addTemporalCallback(FtcLift.TRAVEL_TIME_2_HIGH_BASKET + 100, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .build();

    score1 = follower.pathBuilder()
        .addPath(new BezierLine(pickup1Pose, scorePose))
        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDelivery.run();
        })
        .addTemporalCallback(50, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
          if (PARAMS.executeRobotActions) intakeFlipHorizontal.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 100, () -> {
          if (PARAMS.executeRobotActions) lift2HighBasket.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 200, () -> {
          // In case sample is stuck, evict it.
          if (PARAMS.executeRobotActions) intakeSpinOut.run();
        })
        .build();

    // second yellow
    pickup2 = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup2Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) lift2Low.run();
        })
        .addTemporalCallback(50, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDown.run();
        })
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addTemporalCallback(FtcLift.TRAVEL_TIME_2_HIGH_BASKET, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .build();

    score2 = follower.pathBuilder()
        .addPath(new BezierLine(pickup2Pose, scorePose))
        .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDelivery.run();
        })
        .addTemporalCallback(50, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
          if (PARAMS.executeRobotActions) intakeFlipHorizontal.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 100, () -> {
          if (PARAMS.executeRobotActions) lift2HighBasket.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 200, () -> {
          // In case sample is stuck, evict it.
          if (PARAMS.executeRobotActions) intakeSpinOut.run();
        })
        .build();

    // third yellow
    pickup3 = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup3Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) lift2Low.run();
        })
        .addTemporalCallback(50, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDown.run();
        })
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .addTemporalCallback(FtcLift.TRAVEL_TIME_2_HIGH_BASKET, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .build();

    score3 = follower.pathBuilder()
        .addPath(new BezierLine(pickup3Pose, scorePose))
        .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) intakeFlipDelivery.run();
        })
        .addTemporalCallback(50, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
          if (PARAMS.executeRobotActions) intakeFlipHorizontal.run();
        })
        .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 100, () -> {
          if (PARAMS.executeRobotActions) lift2HighBasket.run();
        })
        .build();

    // park
    parkPath = follower.pathBuilder()
        .addPath(new BezierCurve(scorePose, parkControlPose, parkPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) lift2Low.run();
        })
        .addTemporalCallback(50, () -> {
          if (PARAMS.executeRobotActions) intakeFlipHorizontal.run();
        })
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinStop.run();
        })
        .addTemporalCallback(FtcLift.TRAVEL_TIME_2_HIGH_BASKET, () -> {
          if (PARAMS.executeRobotActions) resetLift.run();
        })
        .build();

    return this;
  }

  /**
   * Executes the autonomous workflow.
   */
  public void execute() {
    FtcLogger.enter();

    // Deliver preloaded sample
    if (!saveAndTest()) return;

    if (PARAMS.executeRobotActions) intakeFlipDown.run();
    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 2200);
      if (PARAMS.executeRobotActions) lift2HighBasketBlocking.run();
      if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
      if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
    }

    // Deliver first yellow sample
    if (!saveAndTest()) return;
    if (PARAMS.deliver1) {
      if (PARAMS.executeTrajectories) runFollower(pickup1, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score1, true, 2500);
      if (PARAMS.executeRobotActions) lift2HighBasketBlocking.run();
      if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
      if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
    }

    // Deliver second yellow sample
    if (!saveAndTest()) return;
    if (PARAMS.deliver2) {
      if (PARAMS.executeTrajectories) runFollower(pickup2, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score2, true, 2500);
      if (PARAMS.executeRobotActions) lift2HighBasketBlocking.run();
      if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
      if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
    }

    // Deliver third yellow sample
    if (!saveAndTest()) return;
    if (PARAMS.deliver3) {
      if (PARAMS.executeTrajectories) runFollower(pickup3, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score3, true, 2500);
      if (PARAMS.executeRobotActions) lift2HighBasketBlocking.run();
      if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
      if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
    }

    // Park
    if (!saveAndTest()) return;
    if (PARAMS.park) {
      if (PARAMS.executeTrajectories) runFollower(parkPath, false, 4000);
      if (PARAMS.executeRobotActions) robot.flag.raise(false);
      if (PARAMS.executeRobotActions) robot.lift.stop();
    }

    FtcLogger.exit();
  }
}
