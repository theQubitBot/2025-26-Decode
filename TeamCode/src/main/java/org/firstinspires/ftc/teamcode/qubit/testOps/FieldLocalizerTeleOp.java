package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.Field.FtcField;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.MatchConfig;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

@Disabled
@TeleOp(group = "TestOp")
public class FieldLocalizerTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  private double lastLoopTime = 1;
  private MatchConfig config;
  private Follower follower;
  private Pose startPose, robotPose;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    config = new MatchConfig();
    config.init(hardwareMap, telemetry, false);
    if (config.allianceColor == AllianceColorEnum.BLUE) {
      if (config.robotPosition == RobotPositionEnum.GOAL) {
        startPose = FtcField.blueGoalStartPose;
      } else {
        startPose = FtcField.blueAudienceStartPose;
      }
    } else {
      if (config.robotPosition == RobotPositionEnum.GOAL) {
        startPose = FtcField.redGoalStartPose;
      } else {
        startPose = FtcField.redAudienceStartPose;
      }
    }

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);
    follower.update();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
    telemetry.update();
    FtcUtils.sleep(FtcUtils.CYCLE_MS);
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    telemetry.addData(FtcUtils.TAG, "Starting...");
    telemetry.update();
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    // Show the elapsed game time and wheel power.
    loopTime.reset();

    follower.updatePose();
    robotPose = follower.getPose();
    telemetry.addData("x", " %.1f", robotPose.getX());
    telemetry.addData("y", " %.1f", robotPose.getY());
    telemetry.addData("heading", "%.1f", Math.toDegrees(robotPose.getHeading()));
    telemetry.addData("total heading", "%.1f", Math.toDegrees(follower.getTotalHeading()));
    telemetry.addLine();
    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        lastLoopTime, runtime.seconds());
    telemetry.update();
    lastLoopTime = loopTime.milliseconds();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
  }
}
