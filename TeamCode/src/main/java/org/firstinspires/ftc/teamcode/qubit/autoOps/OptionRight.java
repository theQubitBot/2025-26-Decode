/* Copyright (c) 2025 The Qubit Bot. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
