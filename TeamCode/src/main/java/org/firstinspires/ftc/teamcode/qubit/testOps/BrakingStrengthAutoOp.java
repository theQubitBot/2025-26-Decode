package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.Field.FtcField;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.concurrent.TimeUnit;

@Disabled
@Config
@Autonomous(group = "TestOp")
public class BrakingStrengthAutoOp extends LinearOpMode {
  public static double brakingStart = Constants.pathConstraints.getBrakingStart();
  public static double brakingStrength = Constants.pathConstraints.getBrakingStrength();
  Pose startPose, scorePose;
  PathChain forwardPathChain, reversePathChain;
  Follower follower;

  @Override
  public void runOpMode() {
    telemetry.addData(FtcUtils.TAG, "Initializing. Please wait...");
    telemetry.update();

    startPose = new Pose(0, 0, FtcField.RADIAN0);
    scorePose = new Pose(48, 0, FtcField.RADIAN0);

    follower = Constants.createFollower(hardwareMap);
    while (opModeInInit()) {
      telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play.");
      telemetry.update();
      FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    Deadline d = new Deadline(5000, TimeUnit.MILLISECONDS);
    while (opModeIsActive()) {
      forwardPathChain = follower.pathBuilder()
          .addPath(new BezierLine(startPose, scorePose))
          .setConstantHeadingInterpolation(startPose.getHeading())
          .setBrakingStrength(brakingStrength)
          .setGlobalDeceleration(brakingStart)
          .build();
      follower.followPath(forwardPathChain, 1, true);
      d.reset();
      do {
        follower.update();
      } while (opModeIsActive() && !d.hasExpired() && follower.isBusy());

      FtcUtils.sleep(FtcUtils.CYCLE_MS);
      reversePathChain = follower.pathBuilder()
          .addPath(new BezierLine(scorePose, startPose))
          .setConstantHeadingInterpolation(startPose.getHeading())
          .setBrakingStrength(brakingStrength)
          .setGlobalDeceleration(brakingStart)
          .build();
      follower.followPath(reversePathChain, 1, true);
      d.reset();
      do {
        follower.update();
      } while (opModeIsActive() && !d.hasExpired() && follower.isBusy());
      FtcUtils.sleep(FtcUtils.CYCLE_MS * 20);
    }
  }
}
