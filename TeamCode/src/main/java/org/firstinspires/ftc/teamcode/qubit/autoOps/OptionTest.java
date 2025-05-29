package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

/**
 * A class to implement autonomous objective
 */
public final class OptionTest extends OptionBase {
  Pose endPose = new Pose(24, 0, RADIAN0);
  PathChain pathChain;

  public OptionTest(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
  }

  public OptionTest init() {
    pathChain = follower.pathBuilder()
        .addBezierLine(new Point(startPose), new Point(endPose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .addTemporalCallback(100, lift2HighBasket)
        .build();
    return this;
  }

  /**
   * Executes the autonomous workflow.
   */
  public void execute() {
    FtcLogger.enter();
    runFollower(pathChain, true, 3000);
    FtcLogger.exit();
  }
}
