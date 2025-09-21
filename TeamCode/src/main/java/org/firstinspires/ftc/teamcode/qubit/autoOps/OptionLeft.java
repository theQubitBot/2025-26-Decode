package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.acmerobotics.dashboard.config.Config;
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
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) intakeSpinStop.run();
        })
        .build();

    // first yellow
    pickup1 = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup1Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
        .addTemporalCallback(200, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score1 = follower.pathBuilder()
        .addPath(new BezierLine(pickup1Pose, scorePose))
        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(200, () -> {
          // In case sample is stuck, evict it.
          if (PARAMS.executeRobotActions) intakeSpinOut.run();
        })
        .build();

    // second yellow
    pickup2 = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup2Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score2 = follower.pathBuilder()
        .addPath(new BezierLine(pickup2Pose, scorePose))
        .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
        .addTemporalCallback(200, () -> {
          // In case sample is stuck, evict it.
          if (PARAMS.executeRobotActions) intakeSpinOut.run();
        })
        .build();

    // third yellow
    pickup3 = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, pickup3Pose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinIn.run();
        })
        .build();

    score3 = follower.pathBuilder()
        .addPath(new BezierLine(pickup3Pose, scorePose))
        .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
        .build();

    // park
    parkPath = follower.pathBuilder()
        .addPath(new BezierCurve(scorePose, parkControlPose, parkPose))
        .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
        .addTemporalCallback(100, () -> {
          if (PARAMS.executeRobotActions) intakeSpinStop.run();
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

    if (PARAMS.deliverPreloaded) {
      if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 2200);
    }

    // Deliver first yellow sample
    if (!saveAndTest()) return;
    if (PARAMS.deliver1) {
      if (PARAMS.executeTrajectories) runFollower(pickup1, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score1, true, 2500);
    }

    // Deliver second yellow sample
    if (!saveAndTest()) return;
    if (PARAMS.deliver2) {
      if (PARAMS.executeTrajectories) runFollower(pickup2, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score2, true, 2500);
    }

    // Deliver third yellow sample
    if (!saveAndTest()) return;
    if (PARAMS.deliver3) {
      if (PARAMS.executeTrajectories) runFollower(pickup3, false, 2600);
      if (PARAMS.executeTrajectories) runFollower(score3, true, 2500);
    }

    // Park
    if (!saveAndTest()) return;
    if (PARAMS.park) {
      if (PARAMS.executeTrajectories) runFollower(parkPath, false, 4000);
    }

    FtcLogger.exit();
  }
}
