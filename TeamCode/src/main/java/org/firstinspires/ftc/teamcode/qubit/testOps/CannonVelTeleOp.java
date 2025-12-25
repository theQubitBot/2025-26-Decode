package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

@Disabled
@TeleOp(group = "TestOp")
public class CannonVelTeleOp extends OpMode {
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  double expectedVelocity;
  final double largeDelta = 100;
  final double smallDelta = 20;
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
    expectedVelocity = FtcCannon.MIN_VELOCITY;
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
    telemetry.addLine();
    if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
      expectedVelocity += largeDelta;
    } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
      expectedVelocity += smallDelta;
    } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
      expectedVelocity -= largeDelta;
    } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
      expectedVelocity -= smallDelta;
    }

    expectedVelocity = Range.clip(expectedVelocity, FtcCannon.MIN_VELOCITY, FtcCannon.MAX_VELOCITY);
    robot.cannon.leftCannonMotor.setVelocity(expectedVelocity);
    robot.cannon.rightCannonMotor.setVelocity(expectedVelocity);

    if (gamepad1.xWasPressed()) {
      robot.cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_UP_POSITION);
    } else if (gamepad1.xWasReleased()) {
      robot.cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_DOWN_POSITION);
    } else if (gamepad1.bWasPressed()) {
      robot.cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_UP_POSITION);
    } else if (gamepad1.bWasReleased()) {
      robot.cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_DOWN_POSITION);
    }

    telemetry.addData("Power", "%.2f",
        robot.cannon.leftCannonMotor.getPower());
    telemetry.addData("Velocity", "expected %.0f, actual %.0f",
        expectedVelocity, robot.cannon.rightCannonMotor.getVelocity());
    telemetry.addData("Camera distance", "%.1f", robot.aprilTag.getGoalDistance());
    telemetry.addData("Localizer distance", "%.1f", robot.localizer.getGoalDistance());

    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
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
