package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

/**
 * A class to implement autonomous objective
 */
@Config
public class OptionRight extends OptionBase {
  // Preloaded Specimen
  Pose specimen1DeliverFinalPose = new Pose(0, 28, RADIAN0);

  Pose specimen2PickFinal = new Pose(46.4, 26.6, RADIAN0);
  Pose specimen2DeliverFinal = new Pose(-1, 25.5, RADIAN180);

  // Park
  public Pose parkFinalPose = new Pose(35, 4, RADIAN180);
  PathChain specimen1DeliveryPath,
      specimen2PickPath, specimen2DeliveryPath,
      parkPath;

  public static class Params {
    public boolean executeTrajectories = true, executeRobotActions = false;
    public boolean deliverSpecimen1 = true,
        deliverSpecimen2 = true,
        park = true;
  }

  public static Params PARAMS = new Params();

  public OptionRight(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
    super(autoOpMode, robot, follower);
    follower.setStartingPose(startPose);
  }

  public OptionRight init() {
    // first specimen
    specimen1DeliveryPath = follower.pathBuilder()
        .addBezierLine(new Point(startPose), new Point(specimen1DeliverFinalPose))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .build();
    specimen1DeliveryPath.name = "specimen1DeliveryPath";

    // pick specimen 2
    specimen2PickPath = follower.pathBuilder()
        .addBezierLine(new Point(specimen1DeliverFinalPose), new Point(specimen2PickFinal))
        .setConstantHeadingInterpolation(startPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) releaseLeftSpecimen.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) lift2Low.run();
        })
        .build();
    specimen2PickPath.name = "specimen2PickPath";

    // deliver specimen 2
    specimen2DeliveryPath = follower.pathBuilder()
        .addBezierLine(new Point(specimen2PickFinal), new Point(specimen2DeliverFinal))
        .setLinearHeadingInterpolation(specimen2PickFinal.getHeading(), specimen2DeliverFinal.getHeading())
        .build();
    specimen2DeliveryPath.name = "specimen2DeliveryPath";

    // park
    parkPath = follower.pathBuilder()
        .addBezierLine(new Point(specimen2DeliverFinal), new Point(parkFinalPose))
        .setConstantHeadingInterpolation(parkFinalPose.getHeading())
        .addTemporalCallback(1, () -> {
          if (PARAMS.executeRobotActions) releaseRightSpecimen.run();
        })
        .addTemporalCallback(10, () -> {
          if (PARAMS.executeRobotActions) lift2Low.run();
        })
        .build();
    parkPath.name = "parkPath";

    return this;
  }

  /**
   * Executes the autonomous workflow.
   */
  public void execute() {
    FtcLogger.enter();

    // Deliver specimen1
    if (!saveAndTest()) return;
    if (PARAMS.executeRobotActions) intakeFlipHorizontal.run();
    if (PARAMS.deliverSpecimen1) {
      if (PARAMS.executeRobotActions) {
        intakeFlipHorizontal.run();
        grabLeftSpecimen.run();
        lift2HighChamber.run();
      }

      if (PARAMS.executeTrajectories) runFollower(specimen1DeliveryPath, true, 3000);

      // Ensure lift has reached correct height
      if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

      // Wait till lift stops swaying
      FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

      // Deliver
      if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
    }

    // Deliver specimen2
    if (!saveAndTest()) return;
    if (PARAMS.deliverSpecimen2) {
      if (PARAMS.executeTrajectories) runFollower(specimen2PickPath, true, 30000);

      if (PARAMS.executeRobotActions) grabRightSpecimen.run();
      if (PARAMS.executeRobotActions) lift2HighChamber.run();
      if (PARAMS.executeTrajectories) runFollower(specimen2DeliveryPath, true, 3000);

      // Ensure lift has reached correct height
      if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

      // Wait till lift stops swaying
      FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

      // Deliver
      if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
    }

    // Park
    if (!saveAndTest()) return;
    if (PARAMS.park) {
      if (PARAMS.executeTrajectories) runFollower(parkPath, false, 3000);
      if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
    }

    FtcLogger.exit();
  }
}
