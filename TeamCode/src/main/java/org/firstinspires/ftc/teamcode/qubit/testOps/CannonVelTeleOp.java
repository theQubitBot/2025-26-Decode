package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.CannonControlData;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

@Disabled
@TeleOp(group = "TestOp")
/// Set the robot at blue goal. Initialize autoOp and then stop it.
/// Then init and start CannonVelTeleOp.
public class CannonVelTeleOp extends OpMode {
  ElapsedTime runtime = null;
  ElapsedTime loopTime = null;
  double expectedVelocity;
  final double largeDelta = 100;
  final double smallDelta = 20;

  double hoodPosition;
  BaseBot robot;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    robot = BaseBot.getBot();
    robot.init(hardwareMap, telemetry, false);
    robot.bulkRead.setCachingMode(LynxModule.BulkCachingMode.AUTO);
    robot.enableTelemetry();
    FtcLogger.exit();
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
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Starting...");
    telemetry.update();
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    robot.start();
    robot.sorter.setStraight(false);
    expectedVelocity = FtcCannon.MIN_VELOCITY;
    hoodPosition = robot.cannon.getHoodPosition();
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    FtcLogger.enter();
    // Show the elapsed game time and wheel power.
    loopTime.reset();

    telemetry.addData("dPad up/down", "Large (%.0f) vel +/-", largeDelta);
    telemetry.addData("dPad left/right", "Small (%.0f) vel +/-", smallDelta);
    telemetry.addData("x/b", "left/right trigger");
    telemetry.addData("y/a", "hood raise/lower");
    telemetry.addLine();

    // cannon
    if (gamepad1.shareWasPressed() || gamepad2.shareWasPressed()) {
      expectedVelocity = 0;
    } else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
      expectedVelocity += largeDelta;
    } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
      expectedVelocity += smallDelta;
    } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
      expectedVelocity -= largeDelta;
    } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
      expectedVelocity -= smallDelta;
    }

    expectedVelocity = Range.clip(expectedVelocity, FtcCannon.MIN_VELOCITY, FtcCannon.MAX_VELOCITY);
    robot.cannon.setVelocity(expectedVelocity, false);

    // triggers
    if (gamepad1.xWasPressed()) {
      robot.cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_UP_POSITION);
    } else if (gamepad1.xWasReleased()) {
      robot.cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_DOWN_POSITION);
    } else if (gamepad1.bWasPressed()) {
      robot.cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_UP_POSITION);
    } else if (gamepad1.bWasReleased()) {
      robot.cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_DOWN_POSITION);
    }

    // hood
    if (gamepad1.y) {
      hoodPosition += FtcServo.LARGE_INCREMENT;
    } else if (gamepad1.a) {
      hoodPosition -= FtcServo.LARGE_INCREMENT;
    }

    hoodPosition = Range.clip(hoodPosition, CannonControlData.HOOD_MIN_POSITION, CannonControlData.HOOD_MAX_POSITION);
    robot.cannon.setHoodPosition(hoodPosition);

    Pose robotPose = robot.localizer.getRobotPose();
    telemetry.addData("robotPose", "x %.1f y %.1f heading %.1f",
        robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
    telemetry.addData("Goal distance", "%.1f", robot.localizer.getGoalDistance());
    telemetry.addData("Velocity", "expected %.0f, actual %.0f %.0f", expectedVelocity,
        robot.cannon.leftCannonMotor.getVelocity(),
        robot.cannon.rightCannonMotor.getVelocity());
    telemetry.addData("Hood", "%.4f", hoodPosition);

    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds", loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    robot.stop();
    this.telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    this.telemetry.update();
    FtcLogger.exit();
  }
}
