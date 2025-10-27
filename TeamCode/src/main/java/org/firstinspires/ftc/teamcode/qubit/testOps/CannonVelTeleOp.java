package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcAprilTag;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.Locale;

@Disabled
@TeleOp(group = "TestOp")
public class CannonVelTeleOp extends OpMode {
  public static double MIN_VELOCITY = 800.0;
  public static double MAX_VELOCITY = 1500.0;
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  double expectedVelocity;
  final double largeDeltaVelocity = 100;
  final double smallDeltaVelocity = 20;
  FtcAprilTag aprilTag;
  FtcCannon cannon;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    if (dashboard != null) {
      telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    aprilTag = new FtcAprilTag(null);
    aprilTag.telemetryEnabled = true;
    aprilTag.init(hardwareMap, telemetry);

    cannon = new FtcCannon(null);
    cannon.telemetryEnabled = true;
    cannon.init(hardwareMap, telemetry);

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
    aprilTag.start();
    cannon.start();
    expectedVelocity = MIN_VELOCITY;
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

    telemetry.addData("dPad up/down", "Large vel +/-");
    telemetry.addData("dPad left/right", "Small vel +/-");
    if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
      expectedVelocity += largeDeltaVelocity;
    } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
      expectedVelocity += smallDeltaVelocity;
    } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
      expectedVelocity -= largeDeltaVelocity;
    } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
      expectedVelocity -= smallDeltaVelocity;
    }

    expectedVelocity = Range.clip(expectedVelocity, MIN_VELOCITY, MAX_VELOCITY);
    cannon.leftCannonMotor.setVelocity(expectedVelocity);
    cannon.rightCannonMotor.setVelocity(expectedVelocity);

    if (gamepad1.xWasPressed()) {
      cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_UP_POSITION);
    } else if (gamepad1.xWasReleased()) {
      cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_DOWN_POSITION);
    } else if (gamepad1.bWasPressed()) {
      cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_UP_POSITION);
    } else if (gamepad1.bWasReleased()) {
      cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_DOWN_POSITION);
    }

    telemetry.addData("Power", String.format(Locale.US, "%.2f",
        cannon.leftCannonMotor.getPower()));
    telemetry.addData("Velocity", String.format(Locale.US, "expected %5.2f, actual %5.2f",
        expectedVelocity, cannon.rightCannonMotor.getVelocity()));
    aprilTag.showTelemetry();

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
    cannon.stop();
    this.telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    this.telemetry.update();
    FtcLogger.exit();
  }
}
