package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.autoOps.OptionBase;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class for Pedro experiments
 */
@Config
public class PedroTestGoal extends OptionBase {
  public Pose control1Pose = new Pose(24, 24, RADIAN0);
  public Pose control2Pose = new Pose(48, 24, RADIAN0);
  public Pose scorePose = new Pose(72, 0, RADIAN0);

  public static double brakingStart = Constants.pathConstraints.getBrakingStart();
  public static double brakingStrength = Constants.pathConstraints.getBrakingStrength();
  PathChain pathChain1, pathChain2;

  public PedroTestGoal(LinearOpMode autoOpMode, BaseBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
  }

  public PedroTestGoal init() {
    // Testing braking strength and early braking on a linear path
    pathChain1 = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .setBrakingStrength(brakingStrength)
        .setGlobalDeceleration(brakingStart)
        .build();

    // Testing curve though given points
    pathChain2 = follower.pathBuilder()
        .addPath(BezierCurve.through(startPose, control1Pose, control2Pose, scorePose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .setBrakingStrength(brakingStrength)
        .setGlobalDeceleration(brakingStart)
        .build();

    return this;
  }

  /**
   * Executes the autonomous workflow.
   */
  public void execute() {
    runFollower(pathChain1, true, 4000);
  }
}
